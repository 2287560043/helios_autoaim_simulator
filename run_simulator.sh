#!/bin/zsh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC="$SCRIPT_DIR/simulator.cpp"
BIN="/tmp/simulator"
OUTPUT_DIR="$SCRIPT_DIR/output"
PARAMS_FILE="$PKG_DIR/autoaim_bring_up/config/node_params.yaml"
HAS_OUTPUT_DIR=0
HAS_PARAMS_FILE=0
FORCE_REBUILD=0

ARGS=("$@")
BIN_ARGS=()
idx=1
while (( idx <= $# )); do
  if [[ "${ARGS[idx]}" == "--output-dir" && $((idx + 1)) -le $# ]]; then
    OUTPUT_DIR="${ARGS[idx + 1]}"
    HAS_OUTPUT_DIR=1
    BIN_ARGS+=("${ARGS[idx]}" "${ARGS[idx + 1]}")
    idx=$((idx + 2))
    continue
  fi
  if [[ "${ARGS[idx]}" == "--help" || "${ARGS[idx]}" == "-h" ]]; then
    BIN_ARGS+=("${ARGS[idx]}")
    idx=$((idx + 1))
    continue
  fi
  if [[ "${ARGS[idx]}" == "--params-file" && $((idx + 1)) -le $# ]]; then
    PARAMS_FILE="${ARGS[idx + 1]}"
    HAS_PARAMS_FILE=1
    BIN_ARGS+=("${ARGS[idx]}" "${ARGS[idx + 1]}")
    idx=$((idx + 2))
    continue
  fi
  if [[ "${ARGS[idx]}" == "--rebuild" ]]; then
    FORCE_REBUILD=1
    idx=$((idx + 1))
    continue
  fi
  BIN_ARGS+=("${ARGS[idx]}")
  idx=$((idx + 1))
done

if (( ! HAS_OUTPUT_DIR )); then
  BIN_ARGS+=(--output-dir "$OUTPUT_DIR")
fi
if (( ! HAS_PARAMS_FILE )); then
  BIN_ARGS+=(--params-file "$PARAMS_FILE")
fi
BUILD_INPUTS=(
  "$0"
  "$SRC"
  "$SCRIPT_DIR/src/node_params.cpp"
  "$SCRIPT_DIR/src/real_generator.cpp"
  "$SCRIPT_DIR/src/autoaim_generator.cpp"
  "$PKG_DIR/autoaim_utilities/src/YawOptimizer.cpp"
  "$PKG_DIR/autoaim_utilities/src/BulletTrajectory.cpp"
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/SimpleTracker.cpp"
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/SingerTracker.cpp"
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/TopTracker.cpp"
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/Top3Tracker.cpp"
  ${(@f)"$(find \
    "$SCRIPT_DIR/include" \
    "$PKG_DIR/autoaim_armor_predictor/include" \
    "$PKG_DIR/autoaim_utilities/include" \
    "$SCRIPT_DIR/dependencies" \
    -type f \( -name '*.hpp' -o -name '*.h' \) -print 2>/dev/null)"}
)

need_rebuild=0
if (( FORCE_REBUILD )) || [[ ! -x "$BIN" ]]; then
  need_rebuild=1
else
  for input in "${BUILD_INPUTS[@]}"; do
    if [[ "$input" -nt "$BIN" ]]; then
      need_rebuild=1
      break
    fi
  done
fi

if (( need_rebuild )); then
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
    -I"$SCRIPT_DIR/dependencies" \
    -I/opt/homebrew/include/eigen3 \
    -I/opt/homebrew/opt/opencv/include/opencv4 \
    -L/opt/homebrew/opt/opencv/lib -lopencv_core \
    -o "$BIN"
fi

"$BIN" "${BIN_ARGS[@]}"
