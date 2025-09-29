import cv2
import numpy as np
import pyttsx3
import sounddevice as sd
import queue
import threading
import whisper
import pyrealsense2 as rs
from ultralytics import YOLO

# =======================
# SETUP
# =======================

# TTS engine with queue
engine = pyttsx3.init()
tts_queue = queue.Queue()

def tts_worker():
    while True:
        text = tts_queue.get()
        if text is None:
            break
        engine.say(text)
        engine.runAndWait()
        tts_queue.task_done()

threading.Thread(target=tts_worker, daemon=True).start()

def speak(text):
    tts_queue.put(text)

# Whisper Model
print("✅ Loading Whisper...")
whisper_model = whisper.load_model("base")

# YOLO Model
print("✅ Loading YOLOv8...")
yolo_model = YOLO("yolov8n.pt")

# Audio Queue
audio_q = queue.Queue()

# State machine for interaction
command_text = ""
awaiting_confirmation = False
last_detected_pose = None
last_detected_label = None

# =======================
# AUDIO CAPTURE
# =======================

def audio_callback(indata, frames, time, status):
    if status:
        print("⚠️ Audio Input Error:", status)
    audio_q.put(indata.copy())

def speech_thread():
    global command_text
    sample_rate = 16000
    with sd.InputStream(samplerate=sample_rate, channels=1, callback=audio_callback):
        print("🎤 Listening continuously...")
        buffer = np.zeros((0, 1), dtype=np.float32)
        while True:
            while not audio_q.empty():
                buffer = np.concatenate((buffer, audio_q.get()), axis=0)

            if len(buffer) > sample_rate * 2:  # 4 sec chunks
                chunk = buffer[:sample_rate*2]
                buffer = buffer[sample_rate*2:]

                audio_fp32 = chunk.flatten().astype(np.float32)
                result = whisper_model.transcribe(audio_fp32, fp16=False, language="en")
                text = result["text"].strip().lower()
                if text:
                    command_text = text
                    print("🗣️ Heard:", command_text)

threading.Thread(target=speech_thread, daemon=True).start()

# =======================
# REALSENSE CAMERA
# =======================

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# Convert (u,v,depth) → (x,y,z)
def pixel_to_point(u, v, depth_value):
    x = (u - intrinsics.ppx) / intrinsics.fx * depth_value
    y = (v - intrinsics.ppy) / intrinsics.fy * depth_value
    z = depth_value
    return x, y, z

# =======================
# MAIN LOOP
# =======================

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # YOLO detection
        results = yolo_model(color_image, verbose=False)
        annotated = color_image.copy()  # start with clean frame

        # If waiting for yes/no → ONLY listen for yes/no
        if awaiting_confirmation:
            if "yes" in command_text:
                speak(f"Attempting to go to {last_detected_label} pose at {last_detected_pose}")
                awaiting_confirmation = False
                command_text = ""
            elif "no" in command_text:
                speak("Okay, mission aborted")
                awaiting_confirmation = False
                command_text = ""
            else:
                # ignore noise until yes/no
                command_text = ""

        # If user command mentions object
        elif command_text:
            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    if conf < 0.5:  # confidence filter
                        continue

                    cls_id = int(box.cls[0])
                    label = yolo_model.names[cls_id].lower()

                    if label in command_text:  # only draw requested class
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                        depth_value = depth_image[cy, cx] * profile.get_device().first_depth_sensor().get_depth_scale()
                        X, Y, Z = pixel_to_point(cx, cy, depth_value)

                        last_detected_pose = (round(X, 3), round(Y, 3), round(Z, 3))
                        last_detected_label = label

                        # Draw only this object
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(annotated, f"{label} {X:.2f},{Y:.2f},{Z:.2f}",
                                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7, (0, 255, 0), 2)
                        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)

                        msg = f"{label} detected at X:{X:.2f}, Y:{Y:.2f}, Z:{Z:.2f} meters. Do you want me to pick it?"
                        print("🤖", msg)
                        speak(msg)

                        awaiting_confirmation = True
                        command_text = ""  # reset after asking
                        break

            cv2.imshow("YOLO + RealSense", annotated)

        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    tts_queue.put(None)
