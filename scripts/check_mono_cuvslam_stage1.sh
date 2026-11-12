#!/usr/bin/env bash
# Stage 1 acceptance: ROS 2, PyCuVSLAM Mono API and real CUDA initialization.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ROS 2 Humble is not installed' >&2; exit 1; }
[[ -e /dev/nvmap ]] || { echo '/dev/nvmap is not visible: run on the Jetson host or expose Jetson GPU devices to the container' >&2; exit 1; }

# ROS binaries and Python packages are built for the Ubuntu 22.04 Python 3.10.
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="${repo_root}/.cache/pycuvslam:${PYTHONPATH:-}"
/usr/bin/python3 - <<'PY'
import rclpy
import cuvslam

print('rclpy:', rclpy.__file__)
print('cuVSLAM:', cuvslam.get_version()[0])
print('Mono mode:', cuvslam.core.Odometry.OdometryMode.Mono)
cuvslam.warm_up_gpu()
print('cuVSLAM CUDA warm-up: OK')
PY
ros2 --help >/dev/null
echo 'Stage 1 passed: ROS 2 and PyCuVSLAM Core can start with CUDA.'
