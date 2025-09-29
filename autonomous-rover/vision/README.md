# 🚀 ISRO Rover Detection using YOLOv8

## 📌 Introduction
This project focuses on detecting **rovers** using a custom-trained **YOLOv8 object detection model**.  
The dataset was created from scratch using a DSLR camera, annotated manually, and augmented to improve robustness.  

This is part of my **portfolio in robotics and computer vision**, showcasing the end-to-end workflow of building a dataset and training a deep learning model for a real-world application.

---

## 📊 Dataset
- **Project ID on Roboflow:** `isro_model`  
- **Source:** DSLR camera images  
- **Annotation Tool:** Roboflow (bounding boxes)  
- **Augmentations:** Rotations, flips, brightness/contrast adjustments, scaling  

📂 [Dataset Link (if public)](https://universe.roboflow.com/)  
📂 If private, a **sample subset** is available in the `/data` folder.  

---

## 🛠️ Model Training
- **Framework:** [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)  
- **Base model used:** `yolov8n.pt` (Nano)  
- **Training configuration:**
  - Image size: `640x640`  
  - Epochs: `50`  
  - Batch size: `16`  
  - Optimizer: `SGD`  

### ⚡ Training Command
```bash
# clone repo
git clone https://github.com/<username>/portfolio.git
cd isro_rover_detection

# install dependencies
pip install -r requirements.txt

# train YOLOv8
yolo detect train data=data.yaml model=yolov8n.pt epochs=50 imgsz=640
