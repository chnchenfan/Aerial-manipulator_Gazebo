#!/usr/bin/env bash

# Source this file from a shell to prepare ROS + Gazebo paths for PX4 SITL.
# Usage:
#   source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced, not executed."
  echo "Use: source /home/cf/PX4_Firmware_clean/ESO_paper_reproduction/src/setup_px4_sitl_ros_env.sh"
  exit 1
fi

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${ROS_WS}/.." && pwd)"
ROS_SETUP="${ROS_WS}/devel/setup.bash"
SITL_BUILD="${REPO_ROOT}/build/px4_sitl_default"

append_ros_package_path() {
  local new_path="$1"

  case ":${ROS_PACKAGE_PATH:-}:" in
    *":${new_path}:"*) ;;
    *) export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+${ROS_PACKAGE_PATH}:}${new_path}" ;;
  esac
}

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ${ROS_SETUP}"
  echo "Run: cd ${ROS_WS} && catkin_make"
  return 1
fi

if [[ ! -d "${SITL_BUILD}" ]]; then
  echo "Missing ${SITL_BUILD}"
  echo "Run: cd ${REPO_ROOT} && DONT_RUN=1 make px4_sitl_default gazebo"
  return 1
fi

source "${ROS_SETUP}"
source "${REPO_ROOT}/Tools/setup_gazebo.bash" "${REPO_ROOT}" "${SITL_BUILD}"

append_ros_package_path "${REPO_ROOT}"
append_ros_package_path "${REPO_ROOT}/Tools/sitl_gazebo"

echo "PX4 SITL ROS environment is ready."
echo "Check: rospack find uav_arm_top"
echo "Check: rospack find mavlink_sitl_gazebo"
