#!/usr/bin/env bash
set -euo pipefail

FER_ROOT="${FER_ROOT:-$HOME/fer_ros2_simulation}"
MUJOCO_DIR="${MUJOCO_DIR:-/home/tim/mujoco-3.9.0-linux-x86_64}"
FER_REPO="${FER_REPO:-https://github.com/GKnerd/fer_ros2_simulation.git}"

if [[ ! -d "$MUJOCO_DIR" ]]; then
  echo "MuJoCo directory not found: $MUJOCO_DIR" >&2
  echo "Set MUJOCO_DIR if your download is somewhere else." >&2
  exit 1
fi

if ! command -v vcs >/dev/null 2>&1; then
  echo "vcstool is required. Install with: pip install vcstool" >&2
  exit 1
fi

if [[ ! -d "$FER_ROOT/.git" ]]; then
  git clone "$FER_REPO" "$FER_ROOT"
fi

mkdir -p "$FER_ROOT/ros2_ws/src"
cd "$FER_ROOT"
vcs import ros2_ws/src < fer_ros2_mujoco.repos

cat <<EOF
FER MuJoCo workspace dependencies imported into:
  $FER_ROOT/ros2_ws/src

Next steps from the FER repository workflow:
  cd $FER_ROOT
  ./.docker/build_image.sh

Then start this pipeline from /workspace/FA_gello with:
  FER_WS=$FER_ROOT/ros2_ws ./start_fer_mujoco_pick_pipeline.sh
EOF
