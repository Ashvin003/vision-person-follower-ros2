# Vision-Based Person Following Robot (ROS 2)

## Overview
This project implements a vision-based person-following system using **YOLOv8** and **ROS 2 Humble** on **TurtleBot3** in Gazebo simulation.

The robot:
- Detects a person using YOLO
- Follows while maintaining ~1 meter distance
- Stops when the person stops
- Searches for the person if lost
- Avoids obstacles using LaserScan
- Can be manually teleoperated

---

## System Architecture
- **Robot**: TurtleBot3 Burger (with camera)
- **Vision**: YOLOv8
- **Simulation**: Gazebo
- **Control**: ROS 2 nodes
- **Obstacle Avoidance**: LaserScan-based logic

---

## Workspace Structure
