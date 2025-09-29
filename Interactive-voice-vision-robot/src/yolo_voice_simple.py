import threading
import whisper
import sounddevice as sd
import numpy as np
import soundfile as sf
from ultralytics import YOLO
import cv2

# ------------------ GLOBALS ------------------
selected_class = None
samplerate = 16000
duration = 3

# ------------------ SPEECH THREAD ------------------
def listen_and_update_class():
    global selected_class
    print("Loading Whisper model...")
    model = whisper.load_model("small")
    print("✅ Whisper loaded!")

    while True:
        print("🎤 Listening...")
        recording = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype="float32")
        sd.wait()
        sf.write("temp.wav", recording, samplerate)

        result = model.transcribe("temp.wav", language="en")
        text = result["text"].strip().lower()
        if text:
            print(f"📝 Heard: {text}")
            selected_class = text  # update filter class

# ------------------ YOLO THREAD ------------------
def run_yolo():
    global selected_class
    model = YOLO("yolov8m.pt")
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model.predict(frame, verbose=False)

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]

                # show only selected class (if set)
                if selected_class is None or selected_class in cls_name.lower():
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{cls_name} {conf:.2f}",
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.9, (0, 255, 0), 2)

        cv2.imshow("YOLO + Voice Filter", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# ------------------ MAIN ------------------
if __name__ == "__main__":
    t1 = threading.Thread(target=listen_and_update_class, daemon=True)
    t1.start()
    run_yolo()
