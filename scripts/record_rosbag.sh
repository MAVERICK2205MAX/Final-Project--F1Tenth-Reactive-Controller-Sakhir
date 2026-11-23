#!/usr/bin/env bash
# Official ROS 2 Bag Recording Script per Final Project Specification
# Records all active simulation topics to rosbags/f1tenth_run_01

set -e
source /opt/ros/humble/setup.bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${WORKSPACE_DIR}/rosbags/f1tenth_run_01"

echo "=========================================================="
echo "  F1Tenth ROS 2 Bag Recording Utility"
echo "  Target Output: ${OUTPUT_DIR}"
echo "=========================================================="

if [ -d "${OUTPUT_DIR}" ]; then
    echo "⚠️  Existing bag folder found. Archiving to f1tenth_run_01_backup_$(date +%s)..."
    mv "${OUTPUT_DIR}" "${OUTPUT_DIR}_backup_$(date +%s)"
fi

mkdir -p "${WORKSPACE_DIR}/rosbags"
cd "${WORKSPACE_DIR}/rosbags"

echo "Recording all topics (-a) to f1tenth_run_01..."
echo "Press [Ctrl + C] to stop recording when finished."
ros2 bag record -a -o f1tenth_run_01
