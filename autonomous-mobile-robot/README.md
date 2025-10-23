# Autonomous Mobile Robot (ROS 2)

## Overview
This project focuses on setting up an **autonomous differential drive mobile robot** in ROS 2, including Gazebo simulation, RViz visualization, and ROS 2 control framework integration.  

The project demonstrates:

- Robot spawning in **Gazebo** and **RViz**  
- Integration of **differential drive** and **LiDAR plugins** in Gazebo  
- Configuration of **ROS 2 control framework** with **diff drive controller** and **joint state broadcaster**  
- Publishing robot state to **TF** for visualization in RViz  

Full autonomous navigation, including **SLAM and Nav2**, is not implemented due to Gazebo segmentation fault issues. For reference, a similar SLAM and navigation demo using TurtleBot is available in the [Sens-O-Ros repository](https://github.com/saianup/Sens-O-ROS/blob/main/README.md).  

## Current Progress
- Spawned the mobile robot successfully in **Gazebo** and **RViz**.  
- Added **differential drive** and **LiDAR** plugins in Gazebo for robot interaction.  
- Configured **ROS 2 control framework**:  
  - Loaded **diff drive controller**  
  - Loaded **joint state broadcaster**  
  - Published robot state to **TF** for visualization in RViz  
- Verified all controllers and TF frames in RViz.  

### Pending / Next Steps
- Resolve **Gazebo segmentation fault** when launching the robot with plugins.  
- Integrate **SLAM Toolbox** and **Nav2** for autonomous navigation once simulation stability is achieved.  
- Implement full autonomous navigation stack in this robot.  

## Tech Stack
- **Robot Simulation:** Gazebo  
- **Visualization:** RViz 2  
- **Control Framework:** ROS 2 Humble, `ros2_control`  
- **Plugins:** Differential Drive, LiDAR  
- **Programming:** Python 3 / ROS 2 launch files  

## Installation & Running
1. **Source ROS 2 environment:**

```bash
source /opt/ros/humble/setup.bash
```

2. **Launch Robot in Gazebo with Plugins:**

```bash
ros2 launch agvrobot_description gazebo.launch.py
```

3. **Launch Robot in Rviz2 with ros2_control controllers:**

```bash
ros2 launch agvrobot_description ros2_control.launch.py
```

**Notes**

The project demonstrates setup, ROS 2 control, and visualization of a mobile robot.

Current limitation is Gazebo segmentation fault when combining world and robot plugins. Resolving this is required to proceed with autonomous navigation implementation.
