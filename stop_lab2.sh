#!/usr/bin/env bash
set -euo pipefail

SESSION="lab2_drone"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Closing Lab 2 tmux session..."
    tmux kill-session -t "$SESSION"
fi

echo "Stopping remaining Gazebo processes..."

for pattern in '^gz sim server' '^gz sim gui'; do
    PIDS=$(pgrep -f "$pattern" || true)

    if [ -n "$PIDS" ]; then
        kill -INT $PIDS
    fi
done

echo "Shutdown requested."
echo "Check that simulation processes have stopped."
