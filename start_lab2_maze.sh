#!/usr/bin/env bash
set -e

SESSION="lab2_drone"

MODE="${1:-variant6}"

case "$MODE" in
    variant6)
        FLIGHT_NODE="flight_variant6_high"
        ;;
    test)
        FLIGHT_NODE="flight_test"
        ;;
    *)
        echo "Usage: $0 [variant6|test]"
        exit 1
        ;;
esac

echo "Selected flight program: $FLIGHT_NODE"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session $SESSION already exists."
    echo "Connect with: tmux attach -t $SESSION"
    exit 1
fi

echo "Starting Lab 2..."

tmux new-session -d -s "$SESSION" -n gazebo

tmux send-keys -t "$SESSION:gazebo" \
    'export LIBGL_ALWAYS_SOFTWARE=1; export GZ_SIM_SYSTEM_PLUGIN_PATH="/usr/local/lib/ardupilot_gazebo:${GZ_SIM_SYSTEM_PLUGIN_PATH}"; export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}"; gz sim -v 4 -r ~/ardupilot_gazebo/worlds/iris_maze_lab2.sdf' Enter

tmux new-window -t "$SESSION" -n sitl
tmux send-keys -t "$SESSION:sitl" \
    'cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-rebuild' Enter

tmux new-window -t "$SESSION" -n dds
tmux send-keys -t "$SESSION:dds" \
    'MicroXRCEAgent udp4 -p 2019' Enter

tmux new-window -t "$SESSION" -n ros
tmux send-keys -t "$SESSION:ros" \
    'source /opt/ros/humble/setup.bash; source ~/fpv_labs/install/setup.bash; echo "ROS 2 ready. Run: ros2 run fpv_lab2 flight_variant6"' Enter

echo "Lab 2 processes started."
echo "Connect with: tmux attach -t $SESSION"
echo "Switch windows with: Ctrl+B, then 0/1/2/3"
echo "Flight program selected: $FLIGHT_NODE"
echo "Waiting for ROS 2 and ArduPilot readiness..."

READY=0

for attempt in $(seq 1 60); do
    if bash -c '
        source /opt/ros/humble/setup.bash
        source ~/fpv_labs/install/setup.bash
        timeout 3 ros2 service type /ap/prearm_check 2>/dev/null |
            grep -q "std_srvs/srv/Trigger"
    '; then
        READY=1
        break
    fi
    sleep 2
done

if [ "$READY" -ne 1 ]; then
    echo "ERROR: ArduPilot ROS 2 service was not ready."
    echo "Check the tmux windows before attempting flight."
    exit 1
fi

echo "ArduPilot ROS 2 service detected."
echo "Waiting for ArduPilot pre-arm approval..."

ARMABLE=0

for attempt in $(seq 1 45); do
    if bash -c '
        source /opt/ros/humble/setup.bash
        source ~/fpv_labs/install/setup.bash
        timeout 5 ros2 service call /ap/prearm_check std_srvs/srv/Trigger "{}" 2>/dev/null
    ' | grep -q 'success=True'; then
        ARMABLE=1
        break
    fi
    sleep 2
done

if [ "$ARMABLE" -ne 1 ]; then
    echo "ERROR: ArduPilot did not become armable."
    exit 1
fi

echo "ArduPilot is armable."
echo "Checking drone state..."

STATUS=$(bash -c '
    source /opt/ros/humble/setup.bash
    source ~/fpv_labs/install/setup.bash
    timeout 10 ros2 topic echo /ap/status --once
') || {
    echo "ERROR: Could not read drone status."
    exit 1
}

if ! grep -qE '^armed: false$' <<< "$STATUS" ||
   ! grep -qE '^flying: false$' <<< "$STATUS"; then
    echo "ERROR: Drone is armed or flying. Flight cancelled."
    exit 1
fi

echo "Drone is disarmed and on the ground."
echo "Starting flight: $FLIGHT_NODE"

tmux new-window -t "$SESSION" -n flight
tmux send-keys -t "$SESSION:flight"     "source /opt/ros/humble/setup.bash; source ~/fpv_labs/install/setup.bash; ros2 run fpv_lab2 $FLIGHT_NODE" Enter

echo "Flight command sent."
echo "View flight log: tmux attach -t $SESSION"

