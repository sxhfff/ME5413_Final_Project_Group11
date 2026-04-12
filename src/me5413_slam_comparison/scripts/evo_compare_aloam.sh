#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BAG_DIR="${PKG_DIR}/bags"
RESULT_DIR="${PKG_DIR}/results"
PLOT_DIR="${RESULT_DIR}/plots"

REF_TRAJ="${1:-${BAG_DIR}/ground_truth_trajectory.tum}"
EST_TRAJ="${2:-${BAG_DIR}/aloam_mapped_trajectory.tum}"
RAW_RESULT_ZIP="${3:-${RESULT_DIR}/aloam_ape_raw.zip}"
RAW_PLOT_PDF="${4:-${PLOT_DIR}/aloam_vs_ground_truth_raw.pdf}"
ALIGNED_RESULT_ZIP="${RESULT_DIR}/aloam_ape_aligned.zip"
ALIGNED_PLOT_PDF="${PLOT_DIR}/aloam_vs_ground_truth_aligned.pdf"

mkdir -p "${RESULT_DIR}" "${PLOT_DIR}"
export MPLBACKEND=Agg
export QT_QPA_PLATFORM=offscreen
rm -f "${RAW_RESULT_ZIP}" "${RAW_PLOT_PDF}" "${ALIGNED_RESULT_ZIP}" "${ALIGNED_PLOT_PDF}"

if ! command -v evo_ape >/dev/null 2>&1 || ! command -v evo_traj >/dev/null 2>&1; then
  echo "evo is not installed. Install it first, for example: pip3 install --user evo" >&2
  exit 1
fi

echo "=== Raw APE (not aligned) ==="
evo_ape tum "${REF_TRAJ}" "${EST_TRAJ}" \
  --plot \
  --plot_mode xy \
  --save_plot "${RAW_PLOT_PDF}" \
  --save_results "${RAW_RESULT_ZIP}"

echo
echo "=== Aligned APE (SE3 aligned) ==="
evo_ape tum "${REF_TRAJ}" "${EST_TRAJ}" \
  --align \
  --correct_scale \
  --plot \
  --plot_mode xy \
  --save_plot "${ALIGNED_PLOT_PDF}" \
  --save_results "${ALIGNED_RESULT_ZIP}"

evo_traj tum "${REF_TRAJ}" "${EST_TRAJ}" \
  --ref="${REF_TRAJ}" \
  --align \
  --plot \
  --plot_mode xy
