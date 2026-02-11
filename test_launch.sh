#!/bin/bash
# =============================================================
# Test Launch Script (with Auto-Reset)
# =============================================================
# Automates the tmux testing setup inside the Docker container.
# 
# This script creates a tmux session with 4 panes:
#   Left:                ros2 launch bt_core bt_engine_launch.py
#   Right-Top:           ros2 launch startup startup_launch.py
#   Right-Middle:        python3 mock_groups_test.py
#   Right-Bottom (small): micro_ros_agent
#
# After you detach (Ctrl+B, D) or the session ends, you will
# be prompted to restart everything or quit.
#
# Usage (inside container):
#   bash scripts/test_launch.sh
# =============================================================

SESSION_NAME="test"

# Source ROS2 workspace if setup file exists
SOURCE_CMD=""
if [ -f "install/setup.bash" ]; then
    SOURCE_CMD="source install/setup.bash && "
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    SOURCE_CMD="source /opt/ros/humble/setup.bash && "
fi

# Source micro_ros workspace if it exists
MICRO_ROS_SOURCE=""
if [ -f "$HOME/micro_ros_ws/install/setup.bash" ]; then
    MICRO_ROS_SOURCE="source $HOME/micro_ros_ws/install/setup.bash && "
fi

cleanup() {
    echo ""
    echo "🧹 Cleaning up..."
    # Kill the tmux session (kills all panes/processes inside)
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null
    # Kill any lingering ROS2 nodes from our launches
    pkill -f "bt_engine" 2>/dev/null
    pkill -f "startup_new" 2>/dev/null
    pkill -f "mock_groups_test" 2>/dev/null
    pkill -f "micro_ros_agent" 2>/dev/null
    # Small delay to let processes terminate
    sleep 1
    echo "Cleanup done."
}

launch() {
    echo ""
    echo "Launching test session..."
    echo "============================================================="

    # Create new tmux session (this becomes the left pane)
    tmux new-session -d -s "$SESSION_NAME" -x 200 -y 50

    # Split right side: create right pane (horizontal split)
    tmux split-window -h -t "$SESSION_NAME"

    # Split right pane into top and bottom (vertical split)
    tmux split-window -v -t "$SESSION_NAME"

    # Split right-bottom pane again for micro_ros (small pane at the very bottom)
    tmux split-window -v -t "$SESSION_NAME" -l 5

    # --- Send commands to each pane ---
    # Pane layout after splits:
    #   0 = Left (bt_engine)
    #   1 = Right-Top (startup)
    #   2 = Right-Middle (mock tester)
    #   3 = Right-Bottom (micro_ros_agent)

    # Pane 0 (Left): bt_engine_launch.py
    tmux send-keys -t "$SESSION_NAME:0.0" "${SOURCE_CMD}ros2 launch bt_core bt_engine_launch.py" C-m

    # Pane 1 (Right-Top): startup_launch.py
    tmux send-keys -t "$SESSION_NAME:0.1" "${SOURCE_CMD}ros2 launch startup startup_launch.py" C-m

    # Pane 2 (Right-Middle): mock_groups_test.py
    tmux send-keys -t "$SESSION_NAME:0.2" "${SOURCE_CMD}python3 src/startup/scripts/mock_groups_test.py" C-m

    # Pane 3 (Right-Bottom): micro_ros_agent
    tmux send-keys -t "$SESSION_NAME:0.3" "${MICRO_ROS_SOURCE}ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/mission" C-m

    # Select the mock tester pane as active
    tmux select-pane -t "$SESSION_NAME:0.2"

    # Attach to the session (blocks until detach or session ends)
    tmux attach -t "$SESSION_NAME"
}

# --- Main loop ---
while true; do
    # Clean up any previous session
    cleanup

    # Launch everything
    launch

    # After detach / session exit, prompt user
    echo ""
    echo "============================================================="
    echo "  Session ended. Press ENTER to restart, or 'q' to quit."
    echo "============================================================="
    read -r input
    if [ "$input" = "q" ] || [ "$input" = "Q" ]; then
        cleanup
        echo "Bye!"
        exit 0
    fi
done
