# Automated Sewage Cleaning Robot

## 📌 Overview

This project presents the design and implementation of an **automated sewage cleaning robot** aimed at eliminating **manual scavenging**. The system integrates:

* **YOLOv8** (trained for sewage/manhole detection)
* **Real-time sensor feedback** (ultrasonic & turbidity sensors, IMU)
* **Pulley-based lifting mechanism**
* **Stepper motor (NEMA 23) with rotating plate**
* **Linear actuator–based five-jaw claw gripper**
* **Vacuum pump with suction pipes**
* **Arduino Mega 2560 as central controller**

The robot detects sewage/manholes using YOLO, moves down a manhole using a pulley system, and performs cleaning tasks automatically using turbidity-based decision-making, sludge mixing, suction pumping, and gripper actuation.


<img width="278" height="487" alt="image" src="https://github.com/user-attachments/assets/f9ac33d9-4f7d-4a9b-a3ef-b30214d14cfe" />


---

## 🚀 Motivation

Manual scavenging is a dangerous and dehumanizing practice, exposing workers to **toxic gases, pathogens, and hazardous sludge**. Despite being banned, it persists in many areas due to lack of affordable alternatives.
Our robot automates sewage cleaning to:

* Eliminate human entry into sewage systems.
* Improve **safety**, **efficiency**, and **hygiene** in waste management.
* Enable scalable solutions for urban waste systems.

---

## ⚙️ System Architecture

### 🔹 Vision & Detection

* **YOLOv8 Model (`best.pt`)**

  * Trained for manhole/sewage detection.
  * Precision: **87.25%**
  * Recall: **83.50%**

### 🔹 Control & Processing

* **Arduino Mega 2560**

  * Handles all sensor inputs and actuator commands.
  * Implements decision-making protocols.

### 🔹 Sensors

* **Ultrasonic sensor** → Detects solid waste and obstacle distance.
* **Turbidity sensor** → Monitors sewage density (0–3000 NTU) to decide depth and sludge-mixing requirement.
* **IMU** → Provides stability feedback for operations.

### 🔹 Actuation

* **Pulley system** → Lowers/raises the robot into the manhole.
* **Linear actuator (150N, 100mm stroke)** → Opens/closes the five-jaw claw.
* **NEMA 23 stepper motor (1.26 Nm torque)** → Rotates mixing plate, driven by TB6560 stepper driver.
* **Vacuum pump with suction pipes** → Pumps sludge and water into external waste collection.

### 🔹 Gripper Mechanism

* **Five-jaw claw assembly** (3D-printed PLA) actuated by linear actuator.
* Mechanism ensures smooth gripping & release.

[Open Source Gripper CAD File](https://www.printables.com/model/277093-mechanical-crane-claw)

### 🔹 Power System

* **11.1V 3300mAh Li-Po battery**
* ~**30 minutes runtime per charge**.

---

### 🔄 Workflow

1. **Detection**: YOLOv8 detects manhole → bot aligns itself.
2. **Deployment**: Bot lowered via pulley into sewage.
3. **Sensing**:

   * Ultrasonic sensor → checks for solid waste.
   * Turbidity sensor → decides whether sludge needs mixing.
4. **Actuation**:

   * Stepper motor rotates mixing plate if sludge is dense.
   * Vacuum pump extracts loosened sludge/water.
   * Linear actuator closes claw → grips solid waste.
5. **Collection**: Waste is lifted out & disposed externally.
6. **Repetition**: Cycle continues until sewage is cleared.

---

## 📊 Performance

* Runtime: **30 minutes per charge**
* Detection accuracy: **87.25% precision, 83.50% recall**
* Suction system prevents **clogging** by mixing sludge dynamically.

---
