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

if [[ -e /config/ChassisTuner ]]; then
  chown -R 1000:1000 /config/ChassisTuner
  chmod +x /config/ChassisTuner/bin/ChassisTuner
fi

# Remove Warning
if [[ -e /run/dbus/pid ]]; then
  rm /run/dbus/pid
fi

exec "$@"