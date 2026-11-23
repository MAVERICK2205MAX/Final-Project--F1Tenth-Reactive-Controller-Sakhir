#!/usr/bin/env bash
# Official ROS 2 Bag Visual Playback Utility with RViz2 & Telemetry Dashboard
# Usage: ./scripts/play_rosbag.sh [bag_number: 1 or 2] [rate] (default: 1, 1.0)

source /opt/ros/humble/setup.bash
source /home/admin1/sim_ws/install/setup.bash 2>/dev/null || true

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHOICE="${1:-1}"
RATE="${2:-1.0}"

if [ "${CHOICE}" == "2" ] || [ "${CHOICE}" == "dynamic" ]; then
    BAG_DIR="${WORKSPACE_DIR}/ROS 2 Bag/02_dynamic_overtaking"
    BAG_NAME="Challenge 2: Dynamic Multi-Robot Overtaking"
else
    BAG_DIR="${WORKSPACE_DIR}/ROS 2 Bag/01_10_laps_complete"
    BAG_NAME="Challenge 1: 10 Laps Complete (Static Obstacles)"
fi

echo "=========================================================="
echo "  F1Tenth ROS 2 Visual Playback with RViz2 & Telemetry"
echo "  Selected: ${BAG_NAME}"
echo "  Directory: ${BAG_DIR}"
echo "  Playback Rate: ${RATE}x"
echo "=========================================================="

if [ ! -d "${BAG_DIR}" ]; then
    echo "❌ Error: Rosbag directory not found at ${BAG_DIR}"
    exit 1
fi

RVIZ_CFG="/home/admin1/sim_ws/src/f1tenth_gym_ros/launch/gym_bridge.rviz"
MAP_YAML="/home/admin1/sim_ws/src/f1tenth_gym_ros/maps/Sakhir_obs_map.yaml"
XACRO_PATH="/home/admin1/sim_ws/src/f1tenth_gym_ros/launch/ego_racecar.xacro"

# Cleanup background processes on exit
PIDS=()
cleanup() {
    echo -e "\nStopping visual playback and closing RViz2..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "1. Launching Map Server for Sakhir track..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="${MAP_YAML}" -p use_sim_time:=false >/dev/null 2>&1 &
PIDS+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p node_names:="['map_server']" -p use_sim_time:=false >/dev/null 2>&1 &
PIDS+=($!)

echo "2. Launching 3D Racecar State Publisher & TF Broadcaster..."
python3 -c "import xacro; print(xacro.process_file('${XACRO_PATH}').toxml())" > /tmp/ego_racecar.urdf 2>/dev/null || true
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/ego_racecar.urdf 2>/dev/null)" --remap /robot_description:=/ego_robot_description >/dev/null 2>&1 &
PIDS+=($!)

"${WORKSPACE_DIR}/scripts/odom_to_tf.py" >/dev/null 2>&1 &
PIDS+=($!)

echo "3. Launching Live Telemetry HUD..."
ros2 run controllers telemetry_gui >/dev/null 2>&1 &
PIDS+=($!)

echo "4. Launching RViz2 Visualizer..."
rviz2 -d "${RVIZ_CFG}" >/dev/null 2>&1 &
PIDS+=($!)

sleep 1.5

echo "----------------------------------------------------------"
echo "  🎬 RViz2 and Telemetry Dashboard Opened on Screen!"
echo "  Controls in this terminal:"
echo "    - SPACE: Pause / Resume"
echo "    - UP/DOWN ARROW: Adjust speed"
echo "    - Ctrl+C: Stop & close all windows"
echo "----------------------------------------------------------"
ros2 bag play "${BAG_DIR}" --rate "${RATE}"
