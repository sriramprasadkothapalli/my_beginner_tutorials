
# Beginner Tutorials - ROS 2 Publisher Node

## Overview
The `beginner_tutorials` package is part of a set of beginner-friendly ROS 2 tutorials. This package contains a simple publisher node called `talker` that publishes a string message to a ROS 2 topic named `/chatter`. The aim is to provide an easy introduction to ROS 2 node communication.

The package is built in **C++** and follows **Google C++ Style Guide** standards with additional modifications. 

## Features
- **Publisher Node (`talker`)**: Publishes a custom string message that increments with each iteration.
- **Static Analysis Tools**: 
  - Linted with **cpplint** for compliance with C++ style guidelines.
  - Analyzed with **clang-tidy** to ensure code quality.

## Prerequisites
- **ROS 2 Humble Hawksbill** (or compatible ROS 2 version) installed.
- **C++17** compiler support.
- Python3 installed for building with ROS 2 (`/usr/bin/python3`).
- **colcon** for building ROS 2 workspaces.

## Assumptions / Dependencies
- The workspace is named `ros2_ws`.
- The package is compatible with **ROS 2 Humble**.
- Dependencies for the package:
  - **rclcpp** (ROS client library for C++)
  - **std_msgs** (Standard message types for ROS)

## Installation
To use this package, clone the repository into your ROS 2 workspace:

1. **Navigate to your ROS 2 workspace source directory**:
   ```bash
   cd ~/ros2_ws/src

2. Install dependencies (if not already installed):
```bash
rosdep install --from-paths src --ignore-src -r -y

```
3. Build the workspace using colcon:
```bash
colcon build --packages-select beginner_tutorials

```

4. Source workspace
```bash
source ~/ros2_ws/install/setup.bash
```

5. Run the Publisher (talker) Node:
```bash
ros2 run beginner_tutorials talker

```


**Running Static Analysis Tools
```bash
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/talker.cpp

```
