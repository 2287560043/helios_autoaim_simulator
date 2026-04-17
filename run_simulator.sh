#!/bin/zsh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC="$SCRIPT_DIR/simulator.cpp"
BIN="/tmp/simulator"
PLOT="$SCRIPT_DIR/plot.py"
OUTPUT_DIR="$SCRIPT_DIR/output"
PARAMS_FILE="$PKG_DIR/autoaim_bring_up/config/node_params.yaml"
SHOW_HELP=0
HAS_OUTPUT_DIR=0
HAS_PARAMS_FILE=0
STAMP="/tmp/simulator_output.stamp"

ARGS=("$@")
idx=1
while (( idx <= $# )); do
  if [[ "${ARGS[idx]}" == "--output-dir" && $((idx + 1)) -le $# ]]; then
    OUTPUT_DIR="${ARGS[idx + 1]}"
    HAS_OUTPUT_DIR=1
  fi
  if [[ "${ARGS[idx]}" == "--help" || "${ARGS[idx]}" == "-h" ]]; then
    SHOW_HELP=1
  fi
  if [[ "${ARGS[idx]}" == "--params-file" && $((idx + 1)) -le $# ]]; then
    PARAMS_FILE="${ARGS[idx + 1]}"
    HAS_PARAMS_FILE=1
  fi
  idx=$((idx + 1))
done

BIN_ARGS=("${ARGS[@]}")
if (( ! HAS_OUTPUT_DIR )); then
  BIN_ARGS+=(--output-dir "$OUTPUT_DIR")
fi
if (( ! HAS_PARAMS_FILE )); then
  BIN_ARGS+=(--params-file "$PARAMS_FILE")
fi

touch "$STAMP"

clang++ -std=c++17 -O2 \
  "$SRC" \
  "$SCRIPT_DIR/src/node_params.cpp" \
  "$SCRIPT_DIR/src/real_generator.cpp" \
  "$SCRIPT_DIR/src/autoaim_generator.cpp" \
  "$PKG_DIR/autoaim_utilities/src/YawOptimizer.cpp" \
  "$PKG_DIR/autoaim_utilities/src/BulletTrajectory.cpp" \
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/SimpleTracker.cpp" \
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/SingerTracker.cpp" \
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/TopTracker.cpp" \
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/Top3Tracker.cpp" \
  -I"$PKG_DIR/autoaim_armor_predictor/include" \
  -I"$PKG_DIR/autoaim_armor_predictor/include/armor_predictor" \
  -I"$PKG_DIR/autoaim_utilities/include" \
  -I"$SCRIPT_DIR/compat" \
  -I/opt/homebrew/include/eigen3 \
  -I/opt/homebrew/opt/opencv/include/opencv4 \
  -L/opt/homebrew/opt/opencv/lib -lopencv_core \
  -o "$BIN"

"$BIN" "${BIN_ARGS[@]}"
if (( SHOW_HELP )); then
  exit 0
fi

CSV_FILES=(${(f)"$(find "$OUTPUT_DIR" -type f -name '*.csv' -newer "$STAMP" -print 2>/dev/null)"})
if (( ${#CSV_FILES[@]} == 0 )); then
  exit 0
fi

for csv_file in "${CSV_FILES[@]}"; do
  MPLCONFIGDIR=/tmp/matplotlib XDG_CACHE_HOME=/tmp python3 "$PLOT" "$csv_file"
done
