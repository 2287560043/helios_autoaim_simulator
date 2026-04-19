#!/bin/zsh
set -euo pipefail
setopt NO_BG_NICE
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
  "$SRC"
  "$SCRIPT_DIR/src/node_params.cpp"
  "$SCRIPT_DIR/src/real_generator.cpp"
  "$SCRIPT_DIR/src/autoaim_generator.cpp"
  # "$PKG_DIR/autoaim_energy_predictor/src/tracker/SmallTracker.cpp"
  # "$PKG_DIR/autoaim_energy_predictor/src/tracker/BigTracker.cpp"
  "$PKG_DIR/autoaim_utilities/src/YawOptimizer.cpp"
  "$PKG_DIR/autoaim_utilities/src/BulletTrajectory.cpp"
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/SimpleTracker.cpp"
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/SingerTracker.cpp"
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/TopTracker.cpp"
  "$PKG_DIR/autoaim_armor_predictor/src/tracker/Top3Tracker.cpp"
)

BUILD_DIR="/tmp/autoaim_simulator_build"
PCH_SRC="$SCRIPT_DIR/include/simulator_pch.hpp"
PCH="$BUILD_DIR/simulator.pch"
PCH_DEP="$BUILD_DIR/simulator.pch.d"
SCRIPT_STAMP="$BUILD_DIR/.run_simulator_stamp"
COMPILE_FLAGS=(
  -std=c++17
  -O3
  -g0
  -DNDEBUG
  -I"$PKG_DIR/autoaim_armor_predictor/include"
  -I"$PKG_DIR/autoaim_armor_predictor/include/armor_predictor"
  # -I"$PKG_DIR/autoaim_energy_predictor/include"
  -I"$PKG_DIR/autoaim_utilities/include"
  -I"$SCRIPT_DIR/dependencies"
  -isystem
  /opt/homebrew/include/eigen3
  -isystem
  /opt/homebrew/opt/opencv/include/opencv4
)
LINK_FLAGS=(
  -L/opt/homebrew/opt/opencv/lib
  -lopencv_core
)
OBJECTS=()
COMPILE_TASKS=()

mkdir -p "$BUILD_DIR"
if [[ ! -f "$SCRIPT_STAMP" || "$0" -nt "$SCRIPT_STAMP" ]]; then
  FORCE_REBUILD=1
fi

deps_need_rebuild() {
  local dep="$1"
  local output="$2"
  local dep_path
  local dep_text
  local deps

  if [[ ! -f "$dep" ]]; then
    return 0
  fi

  dep_text="$(<"$dep")"
  dep_text="${dep_text//\\/ }"
  dep_text="${dep_text#*: }"
  deps=(${=dep_text})
  for dep_path in "${deps[@]}"; do
    dep_path="${dep_path%:}"
    if [[ -z "$dep_path" ]]; then
      continue
    fi
    if [[ "$dep_path" != "$PKG_DIR/"* && "$dep_path" != "$SCRIPT_DIR/dependencies/"* ]]; then
      continue
    fi
    if [[ ! -e "$dep_path" || "$dep_path" -nt "$output" ]]; then
      return 0
    fi
  done
  return 1
}

need_pch=0
if (( FORCE_REBUILD )) || [[ ! -f "$PCH" || ! -f "$PCH_DEP" || "$PCH_SRC" -nt "$PCH" ]]; then
  need_pch=1
elif deps_need_rebuild "$PCH_DEP" "$PCH"; then
  need_pch=1
fi

if (( need_pch )); then
  clang++ "${COMPILE_FLAGS[@]}" -Winvalid-pch -x c++-header -MMD -MP -MF "$PCH_DEP" -MT "$PCH" "$PCH_SRC" -o "$PCH"
fi

prepare_object() {
  local source="$1"
  local rel="${source#$PKG_DIR/}"
  local object="$BUILD_DIR/${rel//\//__}"
  local need_compile=0

  object="${object%.cpp}.o"
  OBJECTS+=("$object")

  if (( FORCE_REBUILD )) || [[ ! -f "$object" || "$source" -nt "$object" || "$PCH" -nt "$object" ]]; then
    need_compile=1
  fi

  if (( need_compile )); then
    COMPILE_TASKS+=("$source|$object")
  fi
}

for input in "${BUILD_INPUTS[@]}"; do
  prepare_object "$input"
done

compile_object() {
  local source="$1"
  local object="$2"
  clang++ "${COMPILE_FLAGS[@]}" -Winvalid-pch -include-pch "$PCH" -c "$source" -o "$object"
}

JOBS="$(sysctl -n hw.logicalcpu 2>/dev/null || true)"
if [[ -z "$JOBS" ]]; then
  JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)"
fi
if [[ -z "$JOBS" ]]; then
  JOBS=4
fi

PIDS=()
if (( ${#COMPILE_TASKS[@]} == 1 )); then
  source="${COMPILE_TASKS[1]%%|*}"
  object="${COMPILE_TASKS[1]#*|}"
  compile_object "$source" "$object"
else
  for task in "${COMPILE_TASKS[@]}"; do
    source="${task%%|*}"
    task="${task#*|}"
    object="$task"
    compile_object "$source" "$object" &
    PIDS+=("$!")
    if (( ${#PIDS[@]} >= JOBS )); then
      wait "${PIDS[1]}"
      PIDS=("${PIDS[@]:1}")
    fi
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid"
  done
fi

need_link=0
if (( FORCE_REBUILD )) || [[ ! -x "$BIN" ]]; then
  need_link=1
else
  for object in "${OBJECTS[@]}"; do
    if [[ "$object" -nt "$BIN" ]]; then
      need_link=1
      break
    fi
  done
fi

if (( need_link )); then
  clang++ "${OBJECTS[@]}" "${LINK_FLAGS[@]}" -o "$BIN"
fi

touch "$SCRIPT_STAMP"

"$BIN" "${BIN_ARGS[@]}"
