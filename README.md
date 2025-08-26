# LGDXRobot2 ROS2

![IMG](img.png)

> Please note that development is primarily done on GitLab: https://gitlab.com/yukaitung/lgdxrobot2-ros2

## Overview

LGDXRobot2 ROS2 is an integration software for the LGDXRobot2 utilising ROS2 ecosystem, especially navigation with the NAV2 stack. It provides examples for both physical robots and simulations. Also, it offers Docker images with ready-to-use ROS2 environment on a web interface.

The project currently supports ROS 2 Jazzy on Ubuntu 24.04 and offers seamless integration with the [LGDXRobot Cloud](https://gitlab.com/yukaitung/lgdxrobot2-cloud).

- [Homepage](https://lgdxrobot.bristolgram.uk/lgdxrobot2/)
- [Documentation](https://docs.lgdxrobot.bristolgram.uk/lgdxrobot2/ros2/)

## Packages

The solution consists of the following packages:

- `lgdxrobot2_agent`: A ROS2 node that drives the robot using ROS2 Topics and Services. It also acts as a client for the LGDXRobot Cloud.
- `lgdxrobot2_bringup`: ROS2 launch files demonstrating how to bringup the robot with NAV2 and LGDXRobot Cloud.
- `lgdxrobot2_description`: A URDF model of the robot.
- `lgdxrobot2_webots`: Simulation configuration and driver for Webots.

## Getting Started

### Full Integration with LGDXRobot Cloud

[https://lgdxrobot.bristolgram.uk/get-started/](https://lgdxrobot.bristolgram.uk/get-started/)

### Docker for Simulation

```bash
docker run --rm -it \
  -e PUID=1000 \
  -e PGID=1000 \
  -p 3000:3000 \
  -p 3001:3001 yukaitung/lgdxrobot2.desktop:latest
```

Visit [http://localhost:3000](http://localhost:3000) to access the web interface. If the terminal is closed, you can right-click the desktop to relaunch it from the menu.

The Docker image has ROS2 and Webots installed, along with the this LGDXRobot2 packages.

### Build from Source

1. [Install ROS2](https://docs.ros.org/en/jazzy/Installation.html)
2. [Install NAV2](https://docs.nav2.org/getting_started/index.html)

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

3. [Install Webots](https://cyberbotics.com/doc/guide/installation-procedure)
4. [Install Webots ROS2 Interface](https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started)
5. Install the following packages:

```bash
sudo apt install libprotobuf-dev libgrpc++-dev protobuf-compiler-grpc 
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-imu-transformer # Optional in simulation
```

6. Build the workspace using `colcon build`

[More documentation](https://docs.lgdxrobot.bristolgram.uk/lgdxrobot2/)

## Notes About Docker

The Docker images provide a ready-to-use environment for running LGDXRobot2 on your local machine. They also offer flexibility in the choice of pre-installed packages. The two main repositories are:

- `yukaitung/lgdxrobot2`: A pre-built image for simulation.
- `yukaitung/lgdxrobot2.desktop`: A pre-built image for simulation with a desktop interface accessible via a web browser.

To pull these images from Docker Hub, you can either use the `latest` tag or specify a particular version number, such as `1.0.0`. You can also use the `latest-lite` tag or `<version>-lite` to pull images without Webots installed, you can run Webots on your host computer instead. Please refer to the [releases](https://gitlab.com/yukaitung/lgdxrobot2-ros2/-/releases) page for version history. All images support both amd64 and arm64 architectures.

There are also additional repositories that include dependencies, which can be used as base images for other ROS2 projects:

- `yukaitung/lgdxrobot2.support`
- `yukaitung/lgdxrobot2.supportdesktop`
- `yukaitung/lgdxrobot2.webots`: An unofficial image for running Webots on arm64 Linux.

## License

This project is licensed under the MIT Licence.

## Extra Dependencies

- [m-explore ROS2 port](https://github.com/robo-friends/m-explore-ros2)
- [hwinfo](https://github.com/lfreist/hwinfo)

## Credits

- [Building Webots on arm64 Linux](https://github.com/up200707458/webots)
- [Docker images for Selkies](https://github.com/linuxserver/docker-baseimage-selkies/)

## Acknowledgements

### ROS 2

> S. Macenski, T. Foote, B. Gerkey, C. Lalancette, W. Woodall, “[Robot Operating System 2: Design, architecture, and uses in the wild](https://www.science.org/doi/10.1126/scirobotics.abm6074).”, Science Robotics vol. 7, May 2022.

> S. Macenski, A. Soragna, M. Carroll, Z. Ge, “[Impact of ROS 2 Node Composition in Robotic Systems](https://arxiv.org/abs/2305.09933)”, IEEE Robotics and Autonomous Letters (RA-L), 2023.

### NAV2

> S. Macenski, F. Martín, R. White, J. Clavero. [The Marathon 2: A Navigation System](https://arxiv.org/abs/2003.00368). IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2020.

> S. Macenski, T. Moore, DV Lu, A. Merzlyakov, M. Ferguson, [From the desks of ROS maintainers: A survey of modern & capable mobile robotics algorithms in the robot operating system 2](https://arxiv.org/pdf/2307.15236), Robotics and Autonomous Systems, 2023

> S. Macenski, M. Booker, J. Wallace, Open-Source, [Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics](https://arxiv.org/abs/2401.13078),

### Selkies

> Kim, S., Isla, D., Hejtmánek, L., et al., Selkies-GStreamer, (2024), GitHub repository, https://github.com/selkies-project/selkies-gstreamer

This project has been developed and is supported in part by the National Research Platform (NRP) and the Cognitive Hardware and Software Ecosystem Community Infrastructure (CHASE-CI) at the University of California, San Diego, by funding from the National Science Foundation (NSF), with awards #1730158, #1540112, #1541349, #1826967, #2138811, #2112167, #2100237, and #2120019, as well as additional funding from community partners, infrastructure utilization from the Open Science Grid Consortium, supported by the National Science Foundation (NSF) awards #1836650 and #2030508, and infrastructure utilization from the Chameleon testbed, supported by the National Science Foundation (NSF) awards #1419152, #1743354, and #2027170. This project has also been funded by the Seok-San Yonsei Medical Scientist Training Program (MSTP) Song Yong-Sang Scholarship, College of Medicine, Yonsei University, the MD-PhD/Medical Scientist Training Program (MSTP) through the Korea Health Industry Development Institute (KHIDI), funded by the Ministry of Health & Welfare, Republic of Korea, and the Student Research Bursary of Song-dang Institute for Cancer Research, College of Medicine, Yonsei University.