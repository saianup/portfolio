# 🤖 Interactive Voice-Controlled Object Picking Robot

## 📌 Overview
This project integrates **speech recognition (Whisper)**, **computer vision (YOLOv8 + Intel RealSense)**, and **robotic manipulation** to create an **interactive system** where a user can simply *speak* a command such as:

> "Pick up the bottle"

The system will:
1. Recognize the spoken command (Whisper).
2. Detect the object using YOLOv8.
3. Estimate its **3D pose** with RealSense depth data.
4. (Planned) Send the detected grasp pose to a manipulator for picking.

---

## 🚀 Current Progress
✅ Implemented continuous **voice recognition** (Whisper).  
✅ Integrated **YOLOv8 object detection**.  
✅ Fused **RealSense depth data** for **3D pose estimation** of detected objects.  
✅ Added **interactive confirmation** with text-to-speech (pyttsx3).  

⚠️ **Pending / Next Steps**  
- Integrate manipulator control for actual pick-and-place.  
- Develop grasp pose planning and ROS2 MoveIt integration.  
- Improve natural conversation flow (multi-command sequences).  

---

## 🛠️ Tech Stack
- **Speech Recognition**: [OpenAI Whisper](https://github.com/openai/whisper)  
- **Object Detection**: [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)  
- **Depth Camera**: Intel RealSense D435  
- **Programming**: Python 3.10  
- **Libraries**: `opencv-python`, `sounddevice`, `pyttsx3`, `numpy`, `pyrealsense2`, `ultralytics`

---
