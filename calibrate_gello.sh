#!/bin/bash

set -e

source ~/miniconda3/etc/profile.d/conda.sh
conda activate polymetis
cd ~/gello_software

PORT="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NM37-if00-port0"
TARGET="$HOME/gello_software/gello/agents/gello_agent.py"

OUTPUT=$(python scripts/gello_get_offset.py \
  --start-joints 0 0 0 -1.57 0 1.57 0 \
  --joint-signs 1 -1 1 -1 1 -1 1 \
  --port "$PORT")

echo "$OUTPUT"

OFFSETS_LINE=$(echo "$OUTPUT" | grep "best offsets function of pi" | head -n 1)
GRIPPER_OPEN=$(echo "$OUTPUT" | grep "gripper open" | awk '{print $NF}' | head -n 1)
GRIPPER_CLOSE=$(echo "$OUTPUT" | grep "gripper close" | awk '{print $NF}' | head -n 1)

if [ -z "$OFFSETS_LINE" ]; then
  echo "Konnte 'best offsets function of pi' nicht finden."
  exit 1
fi

if [ -z "$GRIPPER_OPEN" ] || [ -z "$GRIPPER_CLOSE" ]; then
  echo "Konnte Gripper-Werte nicht finden."
  exit 1
fi

export OFFSETS_LINE
export TARGET
export PORT
export GRIPPER_OPEN
export GRIPPER_CLOSE

python3 <<'PY'
import os
import re
from pathlib import Path

target = Path(os.environ["TARGET"])
port = os.environ["PORT"]
offsets_line = os.environ["OFFSETS_LINE"]
gripper_open = int(round(float(os.environ["GRIPPER_OPEN"])))
gripper_close = int(round(float(os.environ["GRIPPER_CLOSE"])))

m = re.search(r'\[(.*)\]', offsets_line)
if not m:
    raise SystemExit("Offsets konnten nicht aus der Ausgabe extrahiert werden.")

raw_items = [x.strip() for x in m.group(1).split(",")]
offset_lines = [f"            {item}," for item in raw_items]
new_offsets_block = "        joint_offsets=(\n" + "\n".join(offset_lines) + "\n        ),"

text = target.read_text()

pattern = re.compile(
    rf'("{re.escape(port)}": DynamixelRobotConfig\(\n'
    rf'(?:(?!\n    \),).)*?'
    rf'        joint_offsets=\(\n'
    rf'(?:(?!        \),).*\n)*?'
    rf'        \),\n'
    rf'        joint_signs=\([^\)]*\),\n'
    rf'        gripper_config=\([^\)]*\),)',
    re.DOTALL
)

match = pattern.search(text)
if not match:
    raise SystemExit("Panda/Franka-Block in gello_agent.py nicht gefunden.")

old_block = match.group(1)

new_block = re.sub(
    r'        joint_offsets=\(\n(?:(?!        \),).*\n)*?        \),',
    new_offsets_block,
    old_block,
    flags=re.DOTALL
)

new_block = re.sub(
    r'        joint_signs=\([^\)]*\),',
    '        joint_signs=(1, -1, 1, -1, 1, -1, 1),',
    new_block
)

new_block = re.sub(
    r'        gripper_config=\([^\)]*\),',
    f'        gripper_config=(8, {gripper_open}, {gripper_close}),',
    new_block
)

updated = text.replace(old_block, new_block, 1)
target.write_text(updated)

print(f"gello_agent.py aktualisiert: {target}")
print("Neue joint_offsets:")
print(new_offsets_block)
print("joint_signs fest gesetzt auf: (1, -1, 1, -1, 1, -1, 1)")
print(f"Neues gripper_config=(8, {gripper_open}, {gripper_close})")
PY