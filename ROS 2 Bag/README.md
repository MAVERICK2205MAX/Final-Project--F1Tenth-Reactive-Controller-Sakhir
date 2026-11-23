# 📦 ROS 2 Bag Recording: `ROS 2 Bag`

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-3498DB?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Storage](https://img.shields.io/badge/Storage-SQLite3-003B57?style=for-the-badge&logo=sqlite)](https://www.sqlite.org/)
[![Status](https://img.shields.io/badge/Status-Verified_&_Complete-2ECC71?style=for-the-badge)]()
[![Messages](https://img.shields.io/badge/Messages-49,463-orange?style=for-the-badge)]()

**Course:** Autonomous Vehicles — II PAO 2026  
**Assessment:** Final Project — Reactive Controllers Competition in F1Tenth  
**Student / Driver:** ISLAM KAZI MAJHARUL (Islam Qaj Mazarul)  
**Group:** Group 2  
**Assigned Track:** Sakhir International Circuit  
**Contributor & TA:** Winter Delgado (@widegonz)  

---

## 📑 Overview & Dataset Specifications

This directory (`ROS 2 Bag`) contains the official, verified ROS 2 bag recording for the F1Tenth Reactive Controller Competition.

| Specification | Metric Value |
| :--- | :--- |
| **Bag Directory** | `ROS 2 Bag` |
| **Storage Backend** | `sqlite3` (`f1tenth_run_01_0.db3`) |
| **Total Captured Messages** | **49,463 messages** |
| **Recording Duration** | **17.83 seconds (continuous high-frequency telemetry)** |
| **Sensor Topics** | LiDAR (`/scan`), Vehicle Odometry (`/ego_racecar/odom`) |
| **Actuator Topics** | Ackermann Drive (`/drive`) |
| **Transform Trees** | Static & Dynamic TF (`/tf`, `/tf_static`) |
| **Telemetry HUD Topics** | `/car_hud`, `/lap_stats`, `/ftg_hud_marker`, `/ftg_target_marker` |

---

## 🚀 How to Inspect & Play the Bag

### 1. Inspect Bag Metadata
```bash
ros2 bag info "ROS 2 Bag"
```

### 2. Replay at Normal Speed (1.0x)
```bash
ros2 bag play "ROS 2 Bag"
```

### 3. Replay at Half Speed (0.5x) per Specification
```bash
ros2 bag play "ROS 2 Bag" --rate 0.5
```

### 4. Automated Helper Scripts
You can also run the bundled helper scripts from the root directory:
```bash
./scripts/play_rosbag.sh 1.0     # Normal speed playback
./scripts/play_rosbag.sh 0.5     # Half-speed replay
./scripts/record_rosbag.sh       # Record a new run
```
