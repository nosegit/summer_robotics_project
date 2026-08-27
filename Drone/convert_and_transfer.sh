#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Configuration
ROBOT="pi@192.168.1.20"
START_X="-1"
START_Y="-6"
INPUT_FILE="$script_dir/targetLocation.json"
OUTPUT_FILE="$script_dir/targetLocation_robot.json"
REMOTE_FILE="/tmp/targetLocation.json"

# The first command-line argument optionally overrides ROBOT.
ROBOT="${1:-$ROBOT}"

python "$script_dir/convert_to_robot_frame.py" \
  --input "$INPUT_FILE" \
  --output "$OUTPUT_FILE" \
  --start-x "$START_X" \
  --start-y "$START_Y"

python "$script_dir/transfer_and_start_mission.py" \
  --robot "$ROBOT" \
  --result "$OUTPUT_FILE" \
  --remote-file "$REMOTE_FILE"
