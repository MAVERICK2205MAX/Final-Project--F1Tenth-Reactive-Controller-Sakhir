#!/usr/bin/env bash
# Official ROS 2 Bag Playback Utility per Final Project Specification
# Usage: ./play_rosbag.sh [playback_rate] (default: 1.0, half speed: 0.5)

source /opt/ros/humble/setup.bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="${WORKSPACE_DIR}/rosbags/f1tenth_run_01"
RATE="${1:-1.0}"

echo "=========================================================="
echo "  F1Tenth ROS 2 Bag Playback Utility"
echo "  Target Bag: ${BAG_DIR}"
echo "  Playback Rate: ${RATE}x"
echo "=========================================================="

if [ ! -d "${BAG_DIR}" ]; then
    echo "❌ Error: Rosbag directory not found at ${BAG_DIR}"
    exit 1
fi

echo "Inspecting recording metadata with 'ros2 bag info'..."
ros2 bag info "${BAG_DIR}"
echo "----------------------------------------------------------"
echo "Starting playback at --rate ${RATE}..."
ros2 bag play "${BAG_DIR}" --rate "${RATE}"
