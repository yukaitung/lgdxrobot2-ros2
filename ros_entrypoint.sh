#!/bin/bash
set -e

# Setup permissions
chown -R abc:abc /config/.ros
chown -R abc:abc /config/.colcon
chown -R abc:abc /config/lgdx_ws
chown -R abc:abc /config/webots_ws

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/config/webots_ws/install/setup.bash"
source "/config/lgdx_ws/install/setup.bash"
exec "$@"