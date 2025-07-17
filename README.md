# LGDXRobot2 ROS2

> Please note that development is primarily done on GitLab: https://gitlab.com/yukaitung/lgdxrobot2-ros2

LGDXRobot2 ROS2 is an integration layer for the ROS2 ecosystem, with examples for both physical robots and simulations. It currently supports ROS2 Jazzy on Ubuntu 24.04, and integrates with NAV2 and Joy. It also provides seamless integration with the [LGDXRobot Cloud](https://gitlab.com/yukaitung/lgdxrobot2-cloud), offering fast deployment using Docker.

- [Homepage](https://lgdxrobot.bristolgram.uk/lgdxrobot2/)
- [Documentation](https://docs.lgdxrobot.bristolgram.uk/lgdxrobot2/ros2/)

## Packages

The solution consists of the following packages:

- `lgdxrobot2_agent`: A ROS2 node that drives the robot using ROS2 Topics and Services. It also acts as a client for the LGDXRobot Cloud.
- `lgdxrobot2_bringup`: ROS2 launch files demonstrating how to bringup the robot with NAV2 and LGDXRobot Cloud.
- `lgdxrobot2_description`: A URDF model of the robot.
- `lgdxrobot2_webots`: Simulation configuration and driver for Webots.

## Getting Started

### Docker for LGDXRobot Cloud

[Full instructions](https://gitlab.com/yukaitung/lgdxrobot2-cloud#getting-started)

### Build from Source

1. [Install ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
2. [Install NAV2](https://docs.nav2.org/getting_started/index.html)
3. [Install Webots](https://cyberbotics.com/doc/guide/installation-procedure)
4. [Install Webots ROS2 Interface](https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started)
5. Install the following packages:

```bash
sudo apt install libprotobuf-dev libgrpc++-dev protobuf-compiler-grpc
```

6. Build the project:

```bash
mkdir -p ~/lgdx_ws/src
cd ~/lgdx_ws/src
git clone --recurse-submodules https://gitlab.com/yukaitung/lgdxrobot2-ros2
cd ..
colcon build
```

[More documentation](https://docs.lgdxrobot.bristolgram.uk/lgdxrobot2/)

## Notes About Docker

This project has 2 Docker images:

- `yukaitung/lgdxrobot2`: A pre-built image for simulation.
- `yukaitung/lgdxrobot2.support`: A support image with all dependencies included.

To pull `yukaitung/lgdxrobot2` from Docker Hub, you can either use the latest tag or specify a particular version number, such as 1.0.0. Please refer to the [releases](https://gitlab.com/yukaitung/lgdxrobot2-ros2/-/releases) page for the version history. All images support both amd64 and arm64 architectures.

## License

This project is licensed under the MIT Licence.
