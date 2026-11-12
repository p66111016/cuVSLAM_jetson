#!/usr/bin/env bash
# Stage 1: ROS 2 Humble + PyCuVSLAM Core on Jetson AGX Orin / JetPack 6.2.
# Uses system Python 3.10 so rclpy and cuvslam share one interpreter.
# Does not install or modify PyTorch.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin=/usr/bin/python3
py_target="${repo_root}/.cache/pycuvslam"
wheel_dir="${repo_root}/.cache/cuvslam-dist"
wheel_name='cuvslam-17.0.1+cu12-cp310-cp310-manylinux_2_35_aarch64.whl'
wheel_url='https://github.com/nvidia-isaac/cuVSLAM/releases/download/v17.0.1/cuvslam-17.0.1%2Bcu12-cp310-cp310-manylinux_2_35_aarch64.whl'
wheel_sha256='d3ab8db351a16fb763b901738098b7fff6a1c3ada547a3d1c81cddea70731b83'
ros_source_version='1.3.0'
ros_source_deb="ros2-apt-source_${ros_source_version}.jammy_all.deb"
ros_source_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/${ros_source_deb}"

die() { echo "ERROR: $*" >&2; exit 1; }

[[ "$(uname -m)" == aarch64 ]] || die 'requires aarch64 Jetson'
[[ -r /etc/nv_tegra_release ]] || die 'Jetson L4T release file is missing'
grep -q '^# R36 ' /etc/nv_tegra_release || die 'requires JetPack 6 / L4T R36'
[[ "$($python_bin -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')" == 3.10 ]] || die 'requires system Python 3.10'
[[ -x /usr/local/cuda/bin/nvcc ]] || die 'CUDA compiler is missing'

if [[ ! -f "${py_target}/cuvslam/__init__.py" ]]; then
  mkdir -p "${wheel_dir}" "${py_target}"
  if [[ ! -f "${wheel_dir}/${wheel_name}" ]]; then
    curl -fL --retry 3 -o "${wheel_dir}/${wheel_name}" "${wheel_url}"
  fi
  echo "${wheel_sha256}  ${wheel_dir}/${wheel_name}" | sha256sum --check --status || die 'PyCuVSLAM wheel checksum mismatch'
  "$python_bin" -m pip install --no-index --no-deps --target "${py_target}" "${wheel_dir}/${wheel_name}"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Installing ROS 2 Humble through the official ROS apt repository.'
  # sudo prompts for a password when required; run this script in a Jetson terminal.
  sudo apt-get update
  sudo apt-get install -y curl
  mkdir -p "${wheel_dir}"
  curl -fL --retry 3 -o "${wheel_dir}/${ros_source_deb}" "${ros_source_url}"
  sudo dpkg -i "${wheel_dir}/${ros_source_deb}"
  sudo apt-get update
  sudo apt-get install -y ros-humble-ros-base python3-colcon-common-extensions
fi

exec bash "${repo_root}/scripts/check_mono_cuvslam_stage1.sh"
