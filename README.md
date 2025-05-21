# InterIIT Self-Balancing Bike

This repository contains the implementation of a controller for the navigation and balancing of a flywheel-based bike for the competition "Programobot" as part of Inter IIT Tech Symphony 2021.

## Team BotterHeads
- Aryan Rajput
- Ritvik Mahajan
- Sameer Talwar
- Shreshth Mehrotra

## Features
- Self-balancing mechanism using a flywheel
- Autonomous navigation with path planning
- Obstacle detection and avoidance using LiDAR
- GPS-based position estimation
- IMU-based orientation control
- Stand mechanism for stability

## Prerequisites

### System Requirements
- Ubuntu 20.04 or later
- ROS Noetic
- Gazebo 11
- Python 3.8 or later

### Required ROS Packages
- rospy
- sensor_msgs
- geometry_msgs
- std_msgs
- tf
- gazebo_ros
- hector_gazebo_plugins
- gazebo_ros_motors

## Installation

1. **Install ROS Noetic**
   ```bash
   # Add ROS repository
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   
   # Update package list
   sudo apt update
   
   # Install ROS Noetic
   sudo apt install ros-noetic-desktop-full
   ```

2. **Install Dependencies**
   ```bash
   # Install required packages
   sudo apt install python3-pip python3-catkin-tools
   pip3 install numpy
   ```

3. **Setup Workspace**
   ```bash
   # Create and setup catkin workspace
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   
   # Clone the repository
   git clone https://github.com/ritvikmahajan01/InterIIT_SelfBalancingBike_Submission

   # Install custom dependencies
   cd InterIIT_SelfBalancingBike_Submission/BotterHeads/sbb_dependencies
   cp -r hector_gazebo_plugins ~/catkin_ws/src/
   cp -r gazebo_ros_motors-master ~/catkin_ws/src/
   
   # Build the workspace
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Running the Simulation

1. **Launch the Simulation**
   ```bash
   # Source the workspace
   source ~/catkin_ws/devel/setup.bash
   
   # Launch the simulation
   roslaunch inter_iit_sbb_description world.launch
   ```

2. **Start the Controller**
   ```bash
   # In a new terminal
   source ~/catkin_ws/devel/setup.bash
   rosrun inter_iit_sbb_description controller.py
   ```

## Simulation Controls

- The bike will automatically start balancing and navigating towards the goal
- The goal position is set to (15.7, -40.7) in the simulation world
- The bike uses LiDAR for obstacle detection and avoidance
- The flywheel automatically adjusts to maintain balance

## Project Structure

```
BotterHeads/
├── inter_iit_sbb_description/
│   ├── launch/           # Launch files
│   ├── meshes/          # 3D model files
│   ├── params/          # Configuration parameters
│   ├── scripts/         # Control scripts
│   ├── urdf/            # Robot description files
│   ├── world/           # Simulation world files
│   └── xacro/           # XACRO robot description
├── sbb_dependencies/    # Custom ROS packages
└── Supporting document and videos/
```

## Key Components

1. **Controller (`controller.py`)**
   - Implements balancing control using PD controller
   - Handles path planning and obstacle avoidance
   - Processes sensor data from IMU, GPS, and LiDAR

2. **Robot Description**
   - Defined in XACRO files
   - Includes all mechanical components and sensors
   - Configures physical properties and joints

3. **Launch Files**
   - `world.launch`: Main simulation launch file
   - `empty_world.launch`: Base world configuration
