#!/usr/bin/env bash
# Source this file before running the stage 3 ROS nodes.
mono_cuvslam_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${mono_cuvslam_root}/ros2_ws/install/setup.bash"
export PYTHONPATH="${mono_cuvslam_root}/.cache/pycuvslam:${PYTHONPATH:-}"
# The login shell may default to Miniforge Python 3.13; ROS Humble uses 3.10.
export PATH="/usr/bin:${PATH}"
unset mono_cuvslam_root
