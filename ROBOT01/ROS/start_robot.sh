#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose --project-directory "$SCRIPT_DIR")

if ! command -v docker >/dev/null 2>&1; then
    echo "Error: docker is not installed or is not in PATH." >&2
    exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
    echo "Error: tmux is not installed or is not in PATH." >&2
    exit 1
fi

if ! command -v htop >/dev/null 2>&1; then
    echo "Error: htop is not installed or is not in PATH on the host." >&2
    exit 1
fi

TMUX_SESSION="${ROBOT_TMUX_SESSION:-robot}"
if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    echo "Error: tmux session '$TMUX_SESSION' already exists." >&2
    echo "Attach with: tmux attach -t '$TMUX_SESSION'" >&2
    exit 1
fi

echo "Starting server_dev..."
"${COMPOSE[@]}" --profile dev up --detach --no-build server_dev

echo "Waiting for server_dev initialization to finish..."
for _ in {1..120}; do
    if "${COMPOSE[@]}" exec --no-TTY server_dev pgrep -x tail >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

if ! "${COMPOSE[@]}" exec --no-TTY server_dev pgrep -x tail >/dev/null 2>&1; then
    echo "Error: server_dev did not finish initializing within 120 seconds." >&2
    exit 1
fi

echo "Building the ROS workspace..."
"${COMPOSE[@]}" exec --no-TTY server_dev bash -lc \
    'source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build --symlink-install'

echo "Creating tmux session '$TMUX_SESSION'..."
terminal_columns="$(tput cols)"
terminal_lines="$(tput lines)"
tmux new-session -d -x "$terminal_columns" -y "$terminal_lines" \
    -s "$TMUX_SESSION" -n dashboard -c "$SCRIPT_DIR"
trap 'tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true' ERR
camera_pane="$(tmux display-message -p -t "$TMUX_SESSION:dashboard" '#{pane_id}')"
hardware_pane="$(tmux split-window -d -t "$TMUX_SESSION:dashboard" \
    -c "$SCRIPT_DIR" -P -F '#{pane_id}' docker compose exec server_dev bash)"
htop_pane="$(tmux split-window -d -t "$TMUX_SESSION:dashboard" \
    -c "$SCRIPT_DIR" -P -F '#{pane_id}')"
shell_pane="$(tmux split-window -d -t "$TMUX_SESSION:dashboard" \
    -c "$SCRIPT_DIR" -P -F '#{pane_id}' docker compose exec server_dev bash)"

tmux set-option -t "$TMUX_SESSION" mouse on
tmux set-option -t "$TMUX_SESSION" pane-border-status top
tmux set-option -t "$TMUX_SESSION" pane-border-format \
    ' #{pane_title} '
tmux select-pane -t "$camera_pane" -T 'Pi camera RTP (host)'
tmux select-pane -t "$hardware_pane" -T 'Robot hardware (server_dev)'
tmux select-pane -t "$htop_pane" -T 'htop (host)'
tmux select-pane -t "$shell_pane" -T 'Shell (server_dev)'
tmux select-layout -t "$TMUX_SESSION:dashboard" tiled

tmux send-keys -t "$camera_pane" -l \
    'python3 scripts/stream_picamera_rtp.py'
tmux send-keys -t "$camera_pane" Enter

tmux send-keys -t "$hardware_pane" -l \
    'source /root/ros2_ws/install/setup.bash && ros2 launch my_robot_bringup hardware.launch.py'
tmux send-keys -t "$hardware_pane" Enter

tmux send-keys -t "$htop_pane" -l 'htop'
tmux send-keys -t "$htop_pane" Enter

tmux send-keys -t "$shell_pane" -l \
    'source install/setup.bash && clear'
tmux send-keys -t "$shell_pane" Enter
tmux select-pane -t "$hardware_pane"
trap - ERR

if [[ -n "${TMUX:-}" ]]; then
    exec tmux switch-client -t "$TMUX_SESSION"
else
    exec tmux attach-session -t "$TMUX_SESSION"
fi
