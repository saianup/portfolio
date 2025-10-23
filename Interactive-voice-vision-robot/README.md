# Interactive Voice-Controlled Object Picking Robot

## Overview
This project integrates **speech recognition (Whisper)**, **computer vision (YOLOv8 + Intel RealSense)**, and **robotic manipulation** to create an interactive system where a user can issue commands such as:

> "Pick up the bottle"

The system performs the following steps:

1. Recognizes the spoken command using Whisper.  
2. Detects the specified object using YOLOv8.  
3. Estimates the object's **3D pose** using Intel RealSense depth data.  
4. (Planned) Sends the detected grasp pose to a manipulator for pick-and-place operations.  

## Current Progress
- Continuous **voice recognition** implemented using Whisper.  
- Integrated **YOLOv8 object detection**.  
- Fused **RealSense depth data** for **3D pose estimation** of detected objects.  
- Added **interactive confirmation** using text-to-speech (`pyttsx3`).  

### Pending / Next Steps
- Integrate manipulator control for actual pick-and-place tasks.  
- Implement grasp pose planning and ROS 2 MoveIt integration.  
- Enhance natural conversation flow to support multi-command sequences.  

## Tech Stack
- **Speech Recognition:** OpenAI Whisper  
- **Object Detection:** Ultralytics YOLOv8  
- **Depth Camera:** Intel RealSense D435  
- **Programming:** Python  
- **Libraries:** `opencv-python`, `sounddevice`, `pyttsx3`, `numpy`, `pyrealsense2`, `ultralytics`  

## Installation and Running

### Step 1: Install Dependencies
Open a terminal and run the following commands:

```bash
# Update pip
python3 -m pip install --upgrade pip

# Install core dependencies
pip install numpy opencv-python pyttsx3 sounddevice pyrealsense2

# Install Whisper
pip install git+https://github.com/openai/whisper.git

# Install YOLOv8 (Ultralytics)
pip install ultralytics

# Optional: Install PyTorch (CPU version, modify if using GPU)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

```

### Step 2: Run the Python Script

```bash
cd src

python3 voice_yolo_realsense.py

```
