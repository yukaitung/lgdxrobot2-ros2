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
5. Clone the repository and put it under `third-party` in LGDXRobot2-ROS2:  
   ```bash
   git clone https://github.com/lfreist/hwinfo third-party
   ```
6. Install required packages:  
   ```bash
   sudo apt install libprotobuf-dev libgrpc++-dev protobuf-compiler-grpc
   ```

#### Build the Project
After installing all dependencies, clone the project and run:
```bash
colcon build
```

### Usage

To test the joystick and hardware:
```bash
cd lgdx_ws # Navigate to the source code location
. install/setup.bash
ros2 launch lgdxrobot2_mcu joy.launch.py
```

