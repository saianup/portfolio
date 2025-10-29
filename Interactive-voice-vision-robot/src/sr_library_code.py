import cv2
import numpy as np
import queue
import threading
import pyrealsense2 as rs
from ultralytics import YOLO
import speech_recognition as sr
import win32com.client

# =======================
# SETUP
# =======================

# TTS engine with queue using Windows SAPI
tts_queue = queue.Queue()
speaker = win32com.client.Dispatch("SAPI.SpVoice")

def tts_worker():
    while True:
        text = tts_queue.get()
        if text is None:
            break
        try:
            speaker.Speak(text)
        except Exception as e:
            print(f"TTS Error: {e}")
        tts_queue.task_done()

threading.Thread(target=tts_worker, daemon=True).start()

def speak(text):
    tts_queue.put(text)

# YOLO Model
print("✅ Loading YOLOv8...")
yolo_model = YOLO("yolov8n.pt")
print("✅ YOLO loaded successfully!")

# Speech Recognition
recognizer = sr.Recognizer()
command_text = ""
awaiting_confirmation = False
last_detected_pose = None
last_detected_label = None
confirmation_asked = False  # Track if we've already asked for confirmation

# =======================
# SPEECH RECOGNITION THREAD WITH FEEDBACK
# =======================

def speech_thread():
    global command_text
    print("🎤 Voice assistant ready!")
    print("💡 Say: 'where is person' or 'find cell phone'")
    
    while True:
        try:
            with sr.Microphone() as source:
                # Quick noise adjustment
                recognizer.adjust_for_ambient_noise(source, duration=0.5)
                recognizer.energy_threshold = 400
                recognizer.pause_threshold = 0.8
                
                print("🎤 Listening...", end=' ', flush=True)
                
                # Listen for voice command
                audio = recognizer.listen(source, timeout=3, phrase_time_limit=3)
                
                print("🔄 Processing...", end=' ', flush=True)
                
                # Convert speech to text
                text = recognizer.recognize_google(audio, language="en-US").lower()
                
                if text and len(text) > 2:
                    command_text = text
                    print(f"✅ Heard: '{command_text}'")
                else:
                    print("❓ No clear speech")
                    
        except sr.WaitTimeoutError:
            print("⏳ No speech detected", end=' ', flush=True)
            continue
        except sr.UnknownValueError:
            print("❓ Couldn't understand", end=' ', flush=True)
            continue
        except Exception as e:
            print(f"❌ Error: {e}", end=' ', flush=True)
            continue

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
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

def pixel_to_point(u, v, depth_value):
    x = (u - intrinsics.ppx) / intrinsics.fx * depth_value
    y = (v - intrinsics.ppy) / intrinsics.fy * depth_value
    z = depth_value
    return x, y, z

# =======================
# OBJECT DETECTION & MATCHING
# =======================

def find_object_in_command(text):
    """Extract object name from speech text"""
    text = text.lower()
    
    # Map speech to YOLO class names
    object_mapping = {
        'cell phone': 'cell phone', 'cellphone': 'cell phone', 'mobile': 'cell phone', 'phone': 'cell phone',
        'person': 'person', 'human': 'person', 'man': 'person', 'woman': 'person',
        'bottle': 'bottle', 'water bottle': 'bottle',
        'cup': 'cup', 'mug': 'cup',
        'chair': 'chair', 'seat': 'chair',
        'laptop': 'laptop', 'computer': 'laptop',
        'keyboard': 'keyboard', 'mouse': 'mouse', 'book': 'book'
    }
    
    # Check for object names in command
    for keyword, yolo_class in object_mapping.items():
        if keyword in text:
            return yolo_class
    
    return None

# =======================
# MAIN LOOP
# =======================

print("🚀 Starting main loop...")
print("💡 Press 'ESC' to exit\n")

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
        annotated = color_image.copy()

        # If waiting for yes/no confirmation
        if awaiting_confirmation:
            if command_text:
                if "yes" in command_text or "go" in command_text or "pick" in command_text:
                    speak(f"Okay, picking up the {last_detected_label}")
                    print(f"🤖: Okay, picking up the {last_detected_label}")
                    awaiting_confirmation = False
                    command_text = ""
                    confirmation_asked = False
                elif "no" in command_text or "stop" in command_text or "cancel" in command_text:
                    speak("Okay, cancelled")
                    print("🤖: Okay, cancelled")
                    awaiting_confirmation = False
                    command_text = ""
                    confirmation_asked = False
                else:
                    # If it's not a clear yes/no, ask only ONCE and clear the command
                    if not confirmation_asked:
                        print(f"🤖: Please say 'yes' to pick it up or 'no' to cancel")
                        speak("Please say yes to pick it up or no to cancel")
                        confirmation_asked = True
                    # Clear the command so we don't keep repeating
                    command_text = ""

        # Process new voice command (only if not awaiting confirmation)
        elif command_text and not awaiting_confirmation:
            target_object = find_object_in_command(command_text)
            
            if target_object:
                print(f"🔍 Looking for {target_object}...")
                object_found = False
                
                for r in results:
                    for box in r.boxes:
                        conf = float(box.conf[0])
                        if conf < 0.5:
                            continue

                        cls_id = int(box.cls[0])
                        label = yolo_model.names[cls_id].lower()

                        # Check if this is the requested object
                        if label == target_object:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                            if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
                                depth_value = depth_image[cy, cx] * depth_scale
                                
                                if depth_value > 0:
                                    X, Y, Z = pixel_to_point(cx, cy, depth_value)

                                    last_detected_pose = (round(X, 3), round(Y, 3), round(Z, 3))
                                    last_detected_label = label

                                    # Draw bounding box
                                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                    cv2.putText(annotated, f"{label} {X:.2f},{Y:.2f},{Z:.2f}",
                                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                                0.7, (0, 255, 0), 2)
                                    cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)

                                    # Voice output with coordinates
                                    msg = f"{label} detected at X:{X:.1f}, Y:{Y:.1f}, Z:{Z:.1f} meters. Should I pick it up?"
                                    print(f"🤖: {msg}")
                                    speak(msg)

                                    awaiting_confirmation = True
                                    confirmation_asked = False  # Reset for new confirmation
                                    object_found = True
                                    command_text = ""  # Clear command after successful processing
                                    break
                    
                    if object_found:
                        break

                if not object_found:
                    response = f"I cannot see {target_object} right now. Please make sure it's in view."
                    print(f"🤖: {response}")
                    speak(response)
                    command_text = ""  # Clear command after processing
            else:
                response = "Please say an object name like person, cell phone, or bottle"
                print(f"🤖: {response}")
                speak(response)
                command_text = ""  # Clear command after processing

        # Show camera feed
        cv2.imshow("Camera View - Press ESC to exit", annotated)

        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    print("\n🛑 Shutting down...")
    pipeline.stop()
    cv2.destroyAllWindows()
    tts_queue.put(None)
