#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG_PATH="${1:-${PKG_DIR}/bags/aloam_eval.bag}"
OUT_DIR="${2:-${PKG_DIR}/bags}"

mkdir -p "${OUT_DIR}"

python3 "${SCRIPT_DIR}/extract_tum_from_bag.py" \
  --bag "${BAG_PATH}" \
  --odom-topic /gazebo/ground_truth/state="${OUT_DIR}/ground_truth_trajectory.tum" \
  --odom-topic /aft_mapped_to_init="${OUT_DIR}/aloam_mapped_trajectory.tum" \
  --odom-topic /laser_odom_to_init="${OUT_DIR}/aloam_laser_odom_trajectory.tum" \
  --path-topic /aft_mapped_path="${OUT_DIR}/aloam_path_trajectory.tum"
