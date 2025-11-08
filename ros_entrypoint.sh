#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Setup permissions
if [[ -e /config/.ros ]]; then
  chown -R 1000:1000 /config/.ros
fi

if [[ -e /config/.colcon ]]; then
  chown -R 1000:1000 /config/.colcon
fi

if [[ -e /config/lgdx_ws ]]; then
  chown -R 1000:1000 /config/lgdx_ws
  source "/config/lgdx_ws/install/setup.bash"
fi

if [[ -e /config/webots_ws ]]; then
  chown -R 1000:1000 /config/webots_ws
  source "/config/webots_ws/install/setup.bash"
fi

if [[ -e /config/ros2_ws ]]; then
  chown -R 1000:1000 /config/ros2_ws
  source "/config/ros2_ws/install/setup.bash"
fi

# Remove Warning
if [[ -e /run/dbus/pid ]]; then
  rm /run/dbus/pid
fi

exec "$@"