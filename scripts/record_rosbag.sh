#!/usr/bin/env bash
# Official ROS 2 Bag Recording Script per Final Project Specification
# Records all active simulation topics to "ROS 2 Bag"

set -e
source /opt/ros/humble/setup.bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${WORKSPACE_DIR}/ROS 2 Bag"

echo "=========================================================="
echo "  F1Tenth ROS 2 Bag Recording Utility"
echo "  Target Output: ${OUTPUT_DIR}"
echo "=========================================================="

if [ -d "${OUTPUT_DIR}" ]; then
    echo "⚠️  Existing bag folder found. Archiving to ROS_2_Bag_backup_$(date +%s)..."
    mv "${OUTPUT_DIR}" "${WORKSPACE_DIR}/ROS_2_Bag_backup_$(date +%s)"
fi

cd "${WORKSPACE_DIR}"
echo "Recording all topics (-a) into 'ROS 2 Bag'..."
echo "Press [Ctrl + C] to stop recording when finished."
ros2 bag record -a -o "ROS 2 Bag"
