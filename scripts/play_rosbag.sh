#!/usr/bin/env bash
# Official ROS 2 Bag Playback Utility per Final Project Specification
# Usage: ./play_rosbag.sh [bag_number: 1 or 2] [playback_rate] (default: 1, 1.0)

source /opt/ros/humble/setup.bash

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
echo "  F1Tenth ROS 2 Bag Playback Utility"
echo "  Selected: ${BAG_NAME}"
echo "  Directory: ${BAG_DIR}"
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
