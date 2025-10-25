#!/bin/bash
set -e

# Setup permissions
chown -R 1000:1000 /config

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/config/webots_ws/install/setup.bash"
source "/config/lgdx_ws/install/setup.bash"
exec "$@"