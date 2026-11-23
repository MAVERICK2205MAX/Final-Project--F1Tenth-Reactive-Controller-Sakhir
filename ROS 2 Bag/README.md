# 📦 ROS 2 Bag Benchmark Datasets — Final Project (F1Tenth)
**Course:** Autonomous Vehicles (II PAO 2026)  
**Student:** Islam Qaj Mazarul (`ISLAM KAZI MAJHARUL` | GitHub: `MAVERICK2205MAX`)  
**Group:** Group 2 | **Assigned Circuit:** Sakhir International Circuit  

This directory contains two authentic, fully serialized ROS 2 bag recordings documenting the successful completion of both Final Project competition challenges with **zero collisions**.

---

## 📁 Directory Structure

```
ROS 2 Bag/
├── 01_10_laps_complete/          # Challenge 1: 10 Consecutive Laps with 5 Static Obstacles
│   ├── 01_10_laps_complete_0.db3 # SQLite3 database (45.8 MiB, 46,002 messages)
│   └── metadata.yaml             # ROS 2 bag metadata specification
├── 02_dynamic_overtaking/        # Challenge 2: Multi-Robot Dynamic Overtaking
│   ├── 02_dynamic_overtaking_0.db3 # SQLite3 database (7.2 MiB, 7,200 messages)
│   └── metadata.yaml             # ROS 2 bag metadata specification
└── README.md                     # Documentation & playback instructions
```

---

## 🏁 Dataset 01: `01_10_laps_complete`
* **Challenge:** Challenge 1 — Static Obstacle Navigation & Endurance (10 Laps)
* **Circuit:** Sakhir (`Sakhir_obs_map` with 5 static pylon obstacles placed along straights)
* **Performance:** 10 / 10 Laps Completed | **0 Collisions (100% Clean)**
* **Lap Statistics:**
  * Fastest Lap: **76.05 s**
  * Average Lap: **76.63 s**
  * Total Duration: **766.17 s (~12.8 minutes)**
  * Total Messages: **46,002 messages**
* **Recorded Topics:**
  | Topic | Type | Count | Rate | Description |
  | :--- | :--- | :--- | :--- | :--- |
  | `/scan` | `sensor_msgs/msg/LaserScan` | 7,662 | 10 Hz | 2D LiDAR range scan (1080 beams, 270° FOV) |
  | `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | 7,662 | 10 Hz | High-precision chassis odometry & velocity |
  | `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | 7,662 | 10 Hz | Commanded steering angle and longitudinal speed |
  | `/current_lap_time`| `std_msgs/msg/Float32` | 7,662 | 10 Hz | Real-time synchronized lap split timer |
  | `/lap_stats` | `std_msgs/msg/String` | 7,662 | 10 Hz | Formatted telemetry status string |
  | `/car_hud` | `visualization_msgs/msg/Marker` | 7,662 | 10 Hz | RViz floating 3D HUD tag |
  | `/lap_count` | `std_msgs/msg/Int32` | 10 | Event | Completed lap integer (1 through 10) |
  | `/lap_time` | `std_msgs/msg/Float32` | 10 | Event | Individual split time recorded per lap |
  | `/best_lap_time` | `std_msgs/msg/Float32` | 10 | Event | Personal best lap record tracker |

### Inspect & Replay Dataset 01:
```bash
source /opt/ros/humble/setup.bash
# Inspect metadata
ros2 bag info "ROS 2 Bag/01_10_laps_complete"

# Replay at 1x speed (or 2x with -r 2.0)
ros2 bag play "ROS 2 Bag/01_10_laps_complete" --rate 1.0
```

---

## ⚡ Dataset 02: `02_dynamic_overtaking`
* **Challenge:** Challenge 2 — Dynamic Obstacles & Autonomous Robot Overtaking
* **Circuit:** Sakhir Circuit
* **Starting Grid:** 4 Autonomous Vehicles with 5-meter consecutive spacing
  * Robot 0 (Ego): 0.0 m (rear of formation)
  * Robot 1 (Opponent 1): 5.0 m ahead
  * Robot 2 (Opponent 2): 10.0 m ahead
  * Robot 3 (Opponent 3): 15.0 m ahead
* **Overtaking Performance:**
  * Opponent 3 Overtaken: **17.08 s**
  * Opponent 2 Overtaken: **17.65 s**
  * Opponent 1 Overtaken: **18.34 s**
  * Outcome: **All 3 Opponents Cleanly Overtaken | 0 Collisions**
* **Duration:** **44.95 s** | **Total Messages:** **7,200 messages**
* **Recorded Topics:**
  | Topic | Type | Count | Rate | Description |
  | :--- | :--- | :--- | :--- | :--- |
  | `/scan` | `sensor_msgs/msg/LaserScan` | 900 | 20 Hz | Ego vehicle LiDAR sensing dynamic opponents |
  | `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | 900 | 20 Hz | Ego vehicle position and velocity trajectory |
  | `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | 900 | 20 Hz | Reactive control commands during overtakes |
  | `/opp1/odom` | `nav_msgs/msg/Odometry` | 900 | 20 Hz | Opponent 1 trajectory |
  | `/opp2/odom` | `nav_msgs/msg/Odometry` | 900 | 20 Hz | Opponent 2 trajectory |
  | `/opp3/odom` | `nav_msgs/msg/Odometry` | 900 | 20 Hz | Opponent 3 trajectory |
  | `/overtake_status`| `std_msgs/msg/String` | 900 | 20 Hz | Live overtake counter & pass notifications |
  | `/car_hud` | `visualization_msgs/msg/Marker` | 900 | 20 Hz | Visual status banner |

### Inspect & Replay Dataset 02:
```bash
source /opt/ros/humble/setup.bash
# Inspect metadata
ros2 bag info "ROS 2 Bag/02_dynamic_overtaking"

# Replay at 1x speed
ros2 bag play "ROS 2 Bag/02_dynamic_overtaking" --rate 1.0
```

---

## 🏆 Summary Checklist
- [x] **01_10_laps_complete**: 10 clean laps on Sakhir with 5 static obstacles (0 collisions)
- [x] **02_dynamic_overtaking**: 4 robots, 5m spacing, 3 clean overtakes (0 collisions)
- [x] Full ROS 2 Humble SQLite3 compatibility verified with `ros2 bag info`
