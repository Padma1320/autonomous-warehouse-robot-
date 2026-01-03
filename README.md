# Autonomous Warehouse Robot

![Demo](docs/demo.jpg)
*ROS2 Jazzy autonomous mobile robot with SLAM-based navigation and LiDAR obstacle detection*

## Overview
Industrial autonomous mobile robot (AMR) system designed for warehouse navigation and patrol missions. Built with ROS2 Jazzy multi-node architecture integrating Gazebo physics simulation, SLAM Toolbox for real-time mapping, and custom Python mission controller for autonomous decision-making.

## Key Features
- **Autonomous Navigation**: Self-navigating patrol system with velocity-based control logic
- **SLAM Mapping**: Real-time 2D environment mapping using SLAM Toolbox (async mode)
- **Obstacle Detection**: 360° LiDAR sensor integration for collision-free navigation
- **Multi-Node Architecture**: Distributed ROS2 system with Gazebo, SLAM, and custom controller nodes
- **Live Visualization**: Real-time sensor data and transform tree (TF) visualization in RViz2
- **Physics Simulation**: Validated navigation in Gazebo Harmonic hexagon warehouse environment

## How It Works
**System Architecture**: Gazebo Harmonic renders TurtleBot3 in warehouse environment with realistic physics. SLAM Toolbox processes LiDAR scans to build 2D occupancy grid map. Custom Python mission node publishes velocity commands for autonomous patrol. RViz2 displays live LaserScan data, map, and robot pose.

**Autonomous Mission Logic**: Robot executes forward motion until obstacle detected. LiDAR scan analysis triggers turning behavior when obstacles approach threshold. Continuous loop maintains patrol pattern while avoiding collisions. Real-time velocity commands published to /cmd_vel topic.

## Tech Stack
**Software & Frameworks**: ROS2 Jazzy (Robot Operating System LTS) • Python 3 with rclpy library • SLAM Toolbox • Gazebo Harmonic • RViz2

**Hardware Simulation**: TurtleBot3 Burger platform • 360° LiDAR sensor • IMU & Odometry sensors

**Development Environment**: Ubuntu 24.04 LTS • VirtualBox virtualization • Colcon build system

## Installation
```bash
# Install dependencies
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-slam-toolbox ros-jazzy-turtlebot3-gazebo

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# Build workspace
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage
```bash
# Terminal 1: Start Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Run autonomous mission
ros2 run amr_mission mission_node
```

## Project Highlights
Designed for warehouse automation and inventory patrol scenarios • Scalable architecture ready for Nav2 integration • Real-time obstacle detection with low-latency response • Demonstrates core robotics concepts (SLAM, autonomy, sensor fusion)
