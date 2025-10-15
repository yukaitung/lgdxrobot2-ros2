#!/bin/bash
set -e

# Setup permissions
chown -R 1000:1000 /config/.ros
chown -R 1000:1000 /config/.colcon
chown -R 1000:1000 /config/lgdx_ws
chown -R 1000:1000 /config/webots_ws
chown -R 1000:1000 /config/webots

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/config/webots_ws/install/setup.bash"
source "/config/lgdx_ws/install/setup.bash"
exec "$@"