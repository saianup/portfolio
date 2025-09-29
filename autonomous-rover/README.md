# 🤖 Autonomous Rover Project

## 📌 Overview
This project demonstrates the development of an **autonomous rover system** that integrates **computer vision** and **robotics simulation**.  
It is divided into two major modules:  

1. **Vision** – A custom-trained **YOLOv8 model** for rover/environment object detection.  
2. **Simulation** – A **ROS2 + Gazebo** based rover simulation with navigation and motion control.  

The goal is to create a rover capable of **perceiving its environment and moving autonomously**, showcasing a full-stack robotics workflow.

---

## 🎯 Objectives
- Build a **custom dataset** of rover/environment images using DSLR.  
- Train a **YOLOv8-based detection model** to recognize rovers and obstacles.  
- Integrate the detection pipeline into a **ROS2 simulation**.  
- Simulate **autonomous navigation** in Gazebo with object-aware behavior.  

---


---

## 🔍 Vision Module
- **Custom dataset**: Captured with DSLR, annotated & augmented.  
- **Model**: YOLOv8 trained on the dataset (`isro_model`).  
- **Outputs**: Detection results, training plots, mAP scores.  

📂 See [Vision README](vision/README.md) for details.  

---

## 🛠️ Simulation Module
- **Platform**: ROS2 (Humble) + Gazebo + MoveIt2  
- **Rover model**: Custom URDF/Xacro  
- **Features**:
  - Spawn rover in Gazebo world  
  - Control with ROS2 topics (velocity, joint states)  
  - Navigation using MoveIt2 planning  
  - Integration with object detection for decision-making (future scope)  

📂 See [Simulation README](simulation/README.md) for details.  

