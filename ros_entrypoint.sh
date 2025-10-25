#!/bin/bash
set -e

# Setup permissions
if [[ -e /config/.ros ]]; then
  chown -R 1000:1000 /config/.ros
fi

if [[ -e /config/.colcon ]]; then
  chown -R 1000:1000 /config/.colcon
fi

if [[ -e /config/lgdx_ws ]]; then
  chown -R 1000:1000 /config/lgdx_ws
fi

if [[ -e /config/webots_ws ]]; then
  chown -R 1000:1000 /config/webots_ws
fi

if [[ -e /config/webots ]]; then
  chown -R 1000:1000 /config/webots
fi

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/config/webots_ws/install/setup.bash"
source "/config/lgdx_ws/install/setup.bash"

echo "Setup complete"

exec "$@"