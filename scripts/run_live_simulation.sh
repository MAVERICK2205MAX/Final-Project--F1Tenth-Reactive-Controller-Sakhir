#!/usr/bin/env bash
# Official Live Simulation Launcher per Final Project Specification
# Runs the full autonomous Follow-The-Gap controller with RViz2 and Telemetry HUD
# Usage: ./scripts/run_live_simulation.sh [challenge: 1 or 2] (default: 1)

source /opt/ros/humble/setup.bash
source /home/admin1/sim_ws/install/setup.bash

CHALLENGE="${1:-1}"

if [ "${CHALLENGE}" == "2" ] || [ "${CHALLENGE}" == "dynamic" ]; then
    echo "=========================================================="
    echo "  F1Tenth Live Autonomous Race: Challenge 2 (Dynamic Overtake)"
    echo "  Student: Islam Qaj Mazarul | Group 2 | Track: Sakhir"
    echo "=========================================================="
    ros2 launch controllers competition_sakhir_dynamic.launch.py
else
    echo "=========================================================="
    echo "  F1Tenth Live Autonomous Race: Challenge 1 (10 Laps Static)"
    echo "  Student: Islam Qaj Mazarul | Group 2 | Track: Sakhir"
    echo "=========================================================="
    ros2 launch controllers competition_sakhir_static.launch.py
fi
