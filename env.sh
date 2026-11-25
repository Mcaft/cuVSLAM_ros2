#!/bin/bash
# Source this before ros2 launch/run. It wires ROS 2 + the python env that has
# pyrealsense2 (2.58.3) and cuvslam, plus the package's python path.
#
#   source ~/ros2_pkg/cuvslam_bridge/env.sh
#   ros2 launch cuvslam_bridge cuvslam.launch.py

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

export PYTHONPATH="$HOME/msnet2d-venv/lib/python3.10/site-packages:$HOME/ros2_ws/build/cuvslam_bridge:$HOME/ros2_ws/install/cuvslam_bridge/lib/python3.10/site-packages:$PYTHONPATH"
