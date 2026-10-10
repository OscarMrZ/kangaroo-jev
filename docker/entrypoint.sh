#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
# mujoco_ros2_control opens a viewer window; without a display, give it a virtual one.
if [ -z "$DISPLAY" ]; then
  Xvfb :99 -screen 0 1280x720x24 >/dev/null 2>&1 &
  export DISPLAY=:99
fi
exec "$@"
