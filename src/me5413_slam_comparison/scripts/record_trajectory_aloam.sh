#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG_DIR="${PKG_DIR}/bags"
BAG_PATH="${1:-${BAG_DIR}/aloam_eval.bag}"

mkdir -p "${BAG_DIR}"

echo "Recording A-LOAM evaluation bag to ${BAG_PATH}"
rosbag record \
  /gazebo/ground_truth/state \
  /aft_mapped_to_init \
  /laser_odom_to_init \
  /aft_mapped_path \
  /tf \
  /tf_static \
  /mid/points \
  -O "${BAG_PATH}"
