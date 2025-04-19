#!/bin/bash

# ====================================================================
# Script: run.sh
# Description:
#   Launches multiple ROS 2 nodes for TurtleBot4 multirobot map merging and exploration.
#   Runs exploration for a fixed duration.
#   Upon completion, stops exploration, continues running mapping and map merging.
#   Periodically saves the map every 10 minutes.
#
# Usage:
#   chmod +x run.sh
#   ./run.sh
# ====================================================================

# --- Configuration ---

# Number of robots
NUM_ROBOTS=3
export NUM_ROBOTS

# Log files for nodes
EXPLORE_LOG_FILE="explore_lite.log"
MAP_MERGE_LOG_FILE="map_merge.log"
MAP_SAVE_LOG_FILE="map_saver.log"

# Timeout durations (in seconds)
EXPLORATION_DURATION=200    # ~3.3 minutes
MAP_SAVE_INTERVAL=600       # 10 minutes

# Paths (modify these paths as per your workspace)
MAP_SAVE_DIRECTORY="$HOME/ros2_maps"  # Directory to save maps
ROS_DISTRO="humble"                   # Replace with your ROS 2 distribution

# --- Functions ---

# Function to ensure a process is stopped
ensure_process_stopped() {
    local pid=$1
    local name=$2
    local max_attempts=10
    local attempt=1

    while ps -p "$pid" > /dev/null 2>&1; do
        if [ "$attempt" -gt "$max_attempts" ]; then
            echo "Error: Failed to stop $name after $max_attempts attempts."
            return 1
        fi
        echo "Attempting to stop $name (attempt $attempt)..."
        kill "$pid" 2>/dev/null || pkill -f "$name"
        sleep 1
        ((attempt++))
    done
    echo "$name stopped successfully."
    return 0
}

# Function to save the map
save_map() {
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    echo "[$(date)] Saving map to ${MAP_SAVE_DIRECTORY}/map_${TIMESTAMP}"
    mkdir -p "$MAP_SAVE_DIRECTORY"
    ros2 run nav2_map_server map_saver_cli -f "${MAP_SAVE_DIRECTORY}/map_${TIMESTAMP}" --ros-args -r map:=/map > "$MAP_SAVE_LOG_FILE" 2>&1
    if [ $? -eq 0 ]; then
        echo "Map saved successfully at ${MAP_SAVE_DIRECTORY}/map_${TIMESTAMP}.yaml"
    else
        echo "Error: Failed to save map."
    fi
}

# Function to periodically save the map
start_periodic_map_saving() {
    echo "Starting periodic map saving every $(($MAP_SAVE_INTERVAL / 60)) minutes..."
    while true; do
        sleep "$MAP_SAVE_INTERVAL"
        save_map
    done &
    MAP_SAVE_PID=$!
    echo "Map saving process started with PID: $MAP_SAVE_PID"
}

# Function to cleanup processes
cleanup() {
    echo "======= Cleaning Up Processes ======="

    # Kill specific processes by PID if they are set
    for pid in "$EXPLORE_LAUNCH_PID" "$MULTI_MAP_MERGE_PID" "$MAP_MERGE_PID" "$MAP_SAVE_PID"; do
        if [ ! -z "$pid" ] && ps -p "$pid" > /dev/null 2>&1; then
            kill "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
            echo "Terminated process PID: $pid"
        fi
    done

    # List of patterns to kill any remaining processes
    PATTERNS=(
        "ros2 launch multirobot_map_merge"
        "ros2 launch explore_lite"
        "explore_node_"
        "ros2 run nav2_map_server"
    )

    for PATTERN in "${PATTERNS[@]}"; do
        if pkill -f "$PATTERN" 2>/dev/null; then
            echo "Terminated processes matching: $PATTERN"
        fi
    done

    echo "Cleanup complete."
    exit 0
}

# --- Trap Signals for Cleanup ---
trap cleanup SIGINT SIGTERM

# --- Environment Setup ---
export TURTLEBOT4_MODEL='standard'
export IGN_GAZEBO_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot4_gazebo/models"

# Ensure ROS 2 environment is sourced
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    echo "Sourced ROS 2 ${ROS_DISTRO} environment."
else
    echo "Error: ROS 2 distribution '${ROS_DISTRO}' not found."
    exit 1
fi

# --- Launch ROS 2 Nodes ---

# Start multirobot_map_merge node
echo "Starting multirobot_map_merge..."
ros2 launch multirobot_map_merge multi_tb4_simulation_launch.py slam_gmapping:=True use_rviz:=True number_of_robots:=$NUM_ROBOTS > "$MAP_MERGE_LOG_FILE" 2>&1 &
MULTI_MAP_MERGE_PID=$!
echo "multirobot_map_merge PID: $MULTI_MAP_MERGE_PID"

# Wait for 10 seconds to allow initialization
echo "Waiting for 10 seconds for multirobot_map_merge to initialize..."
sleep 10

# Start exploration node
echo "Starting explore_lite..."
ros2 launch explore_lite explore_launch.py num_robots:=$NUM_ROBOTS > "$EXPLORE_LOG_FILE" 2>&1 &
EXPLORE_LAUNCH_PID=$!
echo "explore_lite launch PID: $EXPLORE_LAUNCH_PID"

# Wait for exploration to initialize
echo "Waiting for 5 seconds for exploration to initialize..."
sleep 5

# Start map merge process
echo "Starting map_merge..."
ros2 launch multirobot_map_merge map_merge.launch.py > "$MAP_MERGE_LOG_FILE" 2>&1 &
MAP_MERGE_PID=$!
echo "map_merge PID: $MAP_MERGE_PID"

# --- Run Exploration for Fixed Duration ---

echo "Running exploration for $(($EXPLORATION_DURATION / 60)) minutes..."
sleep "$EXPLORATION_DURATION"

# --- Stop Exploration Nodes ---

echo "Stopping exploration nodes after $(($EXPLORATION_DURATION / 60)) minutes..."

# Terminate the main explore_lite launch process
ensure_process_stopped "$EXPLORE_LAUNCH_PID" "explore_lite"

# Additionally, kill all explore_node_X processes
if pkill -f "explore_node_" 2>/dev/null; then
    echo "Terminated all explore_node_X processes."
fi

# --- Start Periodic Map Saving ---
save_map  # Initial map save
start_periodic_map_saving

# --- Continue Running Mapping and Map Merging ---

echo "Exploration phase completed. Continuing mapping and map merging."
echo "Map will be saved every $(($MAP_SAVE_INTERVAL / 60)) minutes."

# Wait indefinitely to keep the script running and allow periodic map saving
wait
