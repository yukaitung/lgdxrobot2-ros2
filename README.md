# LGDXRobot2-ROS2

ROS2 Integration for LGDXRobot2

### Components

- `lgdxrobot2_bringup` – Easy launch file for ROS2
- `lgdxrobot2_daemon` – Core application for integrating chassis and cloud
- `lgdxrobot2_description` – URDF description
- `lgdxrobot2_webots` – Webots drivers

### Build & Run

This project is developed under ROS2 Jazzy and Ubuntu 24.04. The ROS2 packages for LGDXRobot2 include integrations with the in-house cloud platform and Webots.

#### Prerequisites
1. Install ROS
2. Install Webots
3. Install Webots ROS
4. Install ROS NAV
5. Install required packages:  
   ```bash
   sudo apt install libprotobuf-dev libgrpc++-dev protobuf-compiler-grpc
   sudo apt install ros-jazzy-rtabmap-ros
   sudo apt install ros-jazzy-imu-transformer
   ```

#### Build the Project
After installing all dependencies, clone the project and run:
```bash
git clone --recurse-submodules https://gitlab.com/yukaitung/lgdxrobot2-ros2
colcon build
```

### Usage

To test the joystick and hardware:
```bash
cd lgdx_ws # Navigate to the source code location
. install/setup.bash
ros2 launch lgdxrobot2_mcu joy.launch.py
```

