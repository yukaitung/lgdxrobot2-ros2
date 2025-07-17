#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/user/webots_ws/install/setup.bash"
source "/home/user/lgdx_ws/install/setup.bash" --
exec "$@"