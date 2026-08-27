#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Configuration
ROBOT="pi@144.32.70.209"
ROBOT_PASSWORD="raspberry"
START_X="-1"
START_Y="-6"
INPUT_FILE="$script_dir/targetLocation.json"
OUTPUT_FILE="$script_dir/targetLocation_robot.json"
REMOTE_FILE="/home/pi/summer_robotics_project/ROBOT01/ROS/src/targetLocation.json"

# The first command-line argument optionally overrides ROBOT.
ROBOT="${1:-$ROBOT}"

python "$script_dir/convert_to_robot_frame.py" \
  --input "$INPUT_FILE" \
  --output "$OUTPUT_FILE" \
  --start-x "$START_X" \
  --start-y "$START_Y"

if ! command -v sshpass >/dev/null 2>&1; then
  echo "sshpass is required for password-based SCP." >&2
  exit 1
fi

SSHPASS="$ROBOT_PASSWORD" sshpass -e scp \
  "$OUTPUT_FILE" \
  "$ROBOT:$REMOTE_FILE"
