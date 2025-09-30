# 🤖 Self-Localization of a Mobile Robot using Sensor Fusion

This project implements **self-localization for a differential drive mobile robot** by fusing data from **wheel encoders** and an **IMU (MPU6050)**.  
The goal is to achieve accurate pose estimation in **indoor environments (GPS-denied)** using **sensor fusion with an Extended Kalman Filter (EKF)**.  

The work was developed as part of a mini-project in **Robotics and Automation, MIT Anna University**.

---

## 📌 Concepts Used

- **Self-localization** → The robot estimates its own position `(x, y, θ)` in the environment using onboard sensors.  
- **Odometry (Wheel Encoders)** → Tracks displacement and heading based on wheel rotations.  
- **IMU (Inertial Measurement Unit)** → Provides angular velocity and linear acceleration.  
- **Sensor Fusion (EKF)** → Combines encoders + IMU to reduce drift, slippage, and noise.  
- **ROS 2 + micro-ROS** → Middleware to connect the ESP32 microcontroller with the ROS 2 ecosystem.  

---

## 🛠️ Hardware Setup

- **ESP32** running micro-ROS firmware  
- **MPU6050 IMU** (accelerometer + gyroscope)  
- **DC motors with encoders**  
- **L298N motor driver**  
- **Custom laser-cut acrylic chassis**  
- **Li-ion battery pack (11.1V, 2200 mAh)**  

<img width="897" height="760" alt="image" src="https://github.com/user-attachments/assets/36411076-de22-41c2-9476-a378e8ab4405" />

---

## ⚙️ Software Architecture

1. **ESP32 (micro-ROS node)**  
   - Publishes:
     - `/odom` → raw odometry from wheel encoders  
     - `/imu/data` → IMU accelerometer + gyro readings  
   - Subscribes:
     - `/cmd_vel` → velocity commands to control motors  

2. **ROS 2 Host System**  
   - Runs the **micro-ROS agent** (bridges ESP32 ↔ ROS 2).  
   - Launches **robot_localization (EKF)** to fuse `/odom` and `/imu/data`.  
   - Publishes `/odometry/filtered` → final pose estimate.  

3. **Visualization**  
   - **RViz2** to visualize odometry and fused pose.  
   - **Matplotlib live plotter** (Python script) to compare `/odom` vs `/odometry/filtered`.  

---

## Circuit Diagram 

<img width="987" height="547" alt="image" src="https://github.com/user-attachments/assets/d01c3d5d-98fa-4435-bf49-c2672971dc1c" />


## 🚀 How to Run

### 1. Micro-ROS Agent Setup  
Follow the instructions in the [Sensors Repo README]([https://github.com/saianup/portfolio/tree/main/sensors](https://github.com/saianup/Sens-O-ROS/blob/main/README.md)) to build and run the **micro-ROS agent**.  
This creates the bridge between ESP32 and ROS 2.

```bash
# Example: run micro-ROS agent over serial
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
````

---

### 2. Flash Arduino Code

* Open the `src/finalest.ino` in Arduino IDE.
* Select ESP32 board + correct port.
* Upload the code.

This will start publishing `/odom` and `/imu/data` once the micro-ROS agent is running.

---

### 3. Run EKF (robot_localization)

Use the provided launch file:

```bash
ros2 launch agvrobot_description miniproject.launch.py
```

This starts the `ekf_node` which fuses `/odom` + `/imu/data` → `/odometry/filtered`.

---

### 4. Plot Raw vs Filtered Odometry

Run the Python plotter to visualize performance:

```bash
ros2 run agvrobot_description Odom_Plotter
or run the python file in path
self-localization-robot/src/agvrobot_description/agvrobot_description/__init__.py
```

You will see a live graph:

* 🔴 Red → Raw `/odom`
* 🔵 Blue → Filtered `/odometry/filtered`

---

## 📊 Results

* **Raw Odometry** showed drift and accumulated error.
* **Filtered Odometry (EKF)** was smoother and closer to ground-truth motion.
* RViz visualization confirmed improved trajectory tracking.

Example:

| Source               | Behavior                                  |
| -------------------- | ----------------------------------------- |
| `/odom`              | Drift over time, error due to wheel slip  |
| `/odometry/filtered` | Stable, corrected heading & reduced drift |

## Fused Odometry Plot

<img width="838" height="462" alt="image" src="https://github.com/user-attachments/assets/773830f8-729a-45e2-abf5-b7ffbe6059fa" />

## Metrics Evaluated

<img width="986" height="782" alt="image" src="https://github.com/user-attachments/assets/ef5b8764-099b-4b6b-bdf2-620f87660b6b" />

---

## 📖 References

* [robot_localization package](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
* Madgwick & Kalman Filter papers for sensor fusion
* ROS 2 micro-ROS documentation

---

