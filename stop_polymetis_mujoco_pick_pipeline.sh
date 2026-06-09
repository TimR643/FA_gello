#!/usr/bin/env bash
set -euo pipefail

SESSION="${SESSION:-polymetis_mujoco_pick_pipeline}"

if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

pkill -9 -f "run_server" 2>/dev/null || true
pkill -9 -f "launch_robot.py.*robot_client=.*sim" 2>/dev/null || true

echo "Stopped $SESSION and stale local Polymetis MuJoCo simulator processes."
