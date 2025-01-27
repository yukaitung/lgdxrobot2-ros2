# LGDXRobot2-ROS2

XXX

### Links

*   [LGDXRobot2-MCU](https://gitlab.com/yukaitung/lgdxrobot2-mcu)
*   [LGDXRobot2-ChassisTuner](https://gitlab.com/yukaitung/lgdxrobot2-chassistuner)

# How it works

This software is developed with ROS2.

### Functionality

# Getting started

### Prerequisite

XXX

### Build & Run

1. Install ROS
2. Install Webots
3. Install Webots ROS
4. Install ROS NAV
5. clone https://github.com/lfreist/hwinfo --> third-party
6. Install packages libprotobuf-dev libgrpc++-dev protobuf-compiler-grpc

# Dev notes

### lgdxrobot2_navigation

The `slam_launch.py` in `nav2_bringup` does not map accroding to namespace, so I copied launch files from git.
