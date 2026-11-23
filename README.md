# 🏎️ F1Tenth Final Project: Reactive Controller Competition (Sakhir Circuit)
### Autonomous Vehicles Course (II PAO 2026) — Final Project
**Student:** **Islam Qaj Mazarul** (GitHub: [@MAVERICK2205MAX](https://github.com/MAVERICK2205MAX))  
**Assigned Group:** **Group 2**  
**Assigned Track:** **Sakhir Circuit**  
**Teacher Assistant & Contributor:** Winter Delgado ([@widegonz](https://github.com/widegonz))  
**Course Professor:** Nabih Pico, Ph.D.  
**LMS Submission URL:** [https://class.tu.ac.kr/courses/34717/assignments/803271](https://class.tu.ac.kr/courses/34717/assignments/803271)

---

## 📑 Executive Summary & Table of Contents
1. [Project Overview & Objectives](#-project-overview--objectives)
2. [Reactive Controller Architecture (Follow-The-Gap)](#-reactive-controller-architecture)
   - [LiDAR Preprocessing & 7-Point Gaussian Smoothing](#1-lidar-preprocessing--gaussian-smoothing)
   - [Cartesian Travel Corridor Hazard Detection & Safety Bubble](#2-cartesian-travel-corridor-hazard-detection)
   - [Disparity Extender Algorithm](#3-disparity-extender-algorithm)
   - [Adaptive Gap Selection & Gap Centering](#4-adaptive-gap-selection--centering)
   - [Speed-Scheduled Steering Gain Attenuation & Slew Rate Limiter](#5-speed-scheduled-steering--damping)
   - [Dynamic Curvature & Proximity Speed Profiler](#6-dynamic-curvature--proximity-speed-profiler)
3. [Challenge 1: Static Obstacle Challenge (10 Clean Laps)](#-challenge-1-static-obstacle-challenge)
   - [Map Modification per Class 8 GIMP Guidelines](#map-modification-per-class-8-guidelines)
   - [Benchmark Results: 10 Laps with 0 Collisions](#benchmark-results-challenge-1)
4. [Challenge 2: Dynamic Obstacle Challenge (Robot Overtaking)](#-challenge-2-dynamic-obstacle-challenge-robot-overtaking)
   - [4-Robot Grid Setup (5m Consecutive Spacing)](#4-robot-grid-setup)
   - [Dynamic Overtaking Logic & Multi-Agent Results](#dynamic-overtaking-results)
5. [Challenge 3: ROS 2 Bag Recording & Replay](#-challenge-3-ros-2-bag-recording--replay)
6. [Lap Counter & Lap Timer Implementation](#-lap-counter--lap-timer-implementation)
7. [Repository File & Code Structure](#-repository-file--code-structure)
8. [Installation & Execution Guide](#-installation--execution-guide)
9. [Oral Defense Preparation Summary](#-oral-defense-preparation-summary)

---

## 🎯 Project Overview & Objectives
This repository contains the complete, verified, and competition-ready autonomous racing system for the **F1Tenth Reactive Controller Final Project**. The goal is to design, develop, and benchmark an advanced reactive **Follow-The-Gap (FTG)** navigation controller capable of mastering the demanding **Sakhir Formula 1 Circuit** across three rigorous challenges:
- **Challenge 1 (Static Obstacles):** Complete 10 consecutive laps without colliding with any of the 5 newly placed static obstacles.
- **Challenge 2 (Dynamic Obstacles):** Start at the rear of a 4-car formation with 5-meter spacing, autonomously detect opponents as dynamic obstacles, execute clean overtakes, and finish laps collision-free.
- **Challenge 3 (ROS 2 Bags):** Record all active sensor, odometry, and control topics into `f1tenth_run_01` using `ros2 bag record -a` and verify playback.

---

## 🧠 Reactive Controller Architecture

```
[Raw LiDAR Scan (1080 beams)]
            │
            ▼
[FOV Truncation (±70°)] ──► [7-Point Gaussian Convolution Filter]
            │
            ▼
[Cartesian Hazard Check (|y| ≤ 0.38m, x < 3.0m)] ──► [Safety Bubble Dilation]
            │
            ▼
[Disparity Extender (Δr > 1.1m, forward cone)]
            │
            ▼
[Contiguous Free Gap Identification (r > 0.7m)]
            │
            ▼
[Adaptive Gap Centering: Hazard Mode (60/40) vs Racing Line (20/80)]
            │
            ▼
[Speed-Scheduled Steering Gain] ──► [EMA Filter + Slew Rate Limiter] ──► [Ackermann Drive Output]
```

### 1. LiDAR Preprocessing & Gaussian Smoothing
The F1Tenth LiDAR publishes 1080 ranges across a 270° field of view. The controller:
1. Restricts the processing window to the forward arc $\theta \in [-70^\circ, +70^\circ]$.
2. Replaces invalid readings (`NaN`, $\infty$) and clamps maximum distance to $8.0\text{ m}$.
3. Convolves the array with a 7-point normalized Gaussian kernel:
   $$K = \frac{1}{27} \begin{bmatrix} 1 & 3 & 6 & 7 & 6 & 3 & 1 \end{bmatrix}$$
   This removes high-frequency beam quantization noise and prevents erratic steering twitches.

### 2. Cartesian Travel Corridor Hazard Detection
Traditional FTG algorithms search for the nearest point in the entire scan, which often causes parallel walls at $\pm 70^\circ$ to trigger false safety bubbles and dangerous fishtailing.  
Our controller maps each beam into vehicle Cartesian space:
$$x = r_i \cos(\theta_i), \quad y = r_i \sin(\theta_i)$$
A safety bubble is only projected if an obstacle intersects the vehicle's forward travel corridor:
$$|y| \le W_{\text{safety}} \; (0.38\text{ m}) \quad \text{and} \quad 0.05\text{ m} < x < D_{\text{bubble}} \; (3.0\text{ m})$$
When a hazard is confirmed, a circular safety bubble of radius $R_{\text{bubble}} = 0.28\text{ m}$ is projected around the obstacle index:
$$\Delta \theta = \arctan\left(\frac{R_{\text{bubble}}}{d_{\min}}\right), \quad N_{\text{beams}} = \left\lfloor \frac{\Delta \theta}{\theta_{\text{inc}}} \right\rfloor$$

### 3. Disparity Extender Algorithm
When a foreground obstacle (e.g. static pylon or opponent car) partially occludes the background straightaway, a large range step discontinuity occurs ($\Delta r = |r_{i+1} - r_i| > 1.1\text{ m}$).  
The disparity extender projects a virtual shadow over the deeper beams on the farther side by the angular clearance of the vehicle's chassis, ensuring the car never attempts to apex prematurely into the obstacle's corner.

### 4. Adaptive Gap Selection & Centering
Contiguous free beams ($r > 0.7\text{ m}$) are grouped into viable gaps. We implement **Adaptive Dual-Mode Gap Centering**:
- **Hazard Evasion Mode (Obstacle/Opponent Ahead):**
  $$\text{Target Index} = \text{round}(0.60 \cdot \text{Gap Center} + 0.40 \cdot \text{Deepest Plateau})$$
  This centers the vehicle safely between the obstacle and the outer wall, providing symmetric $\approx 0.75\text{ m}$ clearance on both sides.
- **Clear Track Racing Line Mode:**
  $$\text{Target Index} = \text{round}(0.20 \cdot \text{Gap Center} + 0.80 \cdot \text{Deepest Plateau})$$
  This guides the vehicle aggressively toward the deepest road horizon for high-speed corner apexing.

### 5. Speed-Scheduled Steering & Damping
To eliminate high-speed fishtailing on straights while retaining maximum agility in hairpins:
$$\text{Gain}(v) = \left(\frac{v_{\text{steer\_base}}}{v}\right)^p \quad \text{for } v > v_{\text{steer\_base}}$$
$$\delta_{\text{target}} = \delta_{\text{raw}} \cdot \text{clamp}(\text{Gain}, 0.32, 1.0)$$
The steering angle is then smoothed with an Exponential Moving Average (EMA, $\alpha = 0.32$) and bounded by a physical slew rate limiter ($\Delta \delta_{\max} = 4.0\text{ rad/s} \cdot \Delta t$).

### 6. Dynamic Curvature & Proximity Speed Profiler
Speed is modulated automatically based on steering demand and forward clearance:
$$v_{\text{steer}} = v_{\min} + (v_{\max} - v_{\min}) \cdot \left[ \max\left(0, 1 - \frac{|\delta|}{22^\circ}\right) \right]^{1.2}$$
$$v_{\text{dist}} = v_{\min} + (v_{\max} - v_{\min}) \cdot \text{clamp}\left(\frac{d_{\text{clearance}} - 1.8}{4.0}, 0, 1\right)$$
$$v_{\text{final}} = \min(v_{\text{steer}}, v_{\text{dist}})$$

---

## 🚧 Challenge 1: Static Obstacle Challenge

### Map Modification per Class 8 Guidelines
Following the instructions presented in **AV - Class 8** by Prof. Nabih Pico and TA Winter Delgado:
1. `Sakhir_map.png` was loaded into GIMP.
2. In ROS occupancy grids, pixel `255` is free space, and `0` (pure black) represents obstacles.
3. 5 circular pylons ($r = 3\text{ px} \approx 0.17\text{ m}$, diameter $0.34\text{ m}$) were added with a $0.40\text{ m}$ offset on 5 long straightaway sectors ($\kappa < 0.008$):
   - **Obstacle 1 (idx 75):** Pit straight exit ($x=0.99, y=30.65$)
   - **Obstacle 2 (idx 250):** North straight ($x=42.16, y=52.96$)
   - **Obstacle 3 (idx 490):** Infield straight ($x=23.38, y=25.21$)
   - **Obstacle 4 (idx 630):** Back straight ($x=16.24, y=12.18$)
   - **Obstacle 5 (idx 915):** South connector straight ($x=27.93, y=-24.77$)
4. Saved as `Sakhir_obs_map.png` and configured in `Sakhir_obs_map.yaml`.

### Benchmark Results (Challenge 1)

| Lap # | Split Time (s) | Best Lap (s) | Cumulative (s) | Obstacle Status | Collision Count | Status |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Lap 01** | 82.06 s | 82.06 s | 82.06 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 02** | 81.75 s | 81.75 s | 163.81 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 03** | 81.82 s | 81.75 s | 245.63 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 04** | **81.50 s** | **81.50 s** | 327.13 s | 5 Static Pylons | 0 | ⭐ **Fastest Lap** |
| **Lap 05** | 81.90 s | 81.50 s | 409.03 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 06** | 81.81 s | 81.50 s | 490.84 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 07** | 82.19 s | 81.50 s | 573.03 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 08** | 81.86 s | 81.50 s | 654.89 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 09** | 81.69 s | 81.50 s | 736.58 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 10** | 81.63 s | 81.50 s | 818.21 s | 5 Static Pylons | 0 | ✅ Clean & Parked |

- **Official Benchmark:** **10 / 10 Clean Laps completed with ZERO collisions**.
- **Fastest Lap:** **81.50 s** | **Average Lap:** **81.82 s**.

---

## 🏎️ Challenge 2: Dynamic Obstacle Challenge (Robot Overtaking)

### 4-Robot Grid Setup
Per the competition specification, 4 autonomous vehicles were placed consecutively along the Sakhir centerline with 5-meter spacing:
- **Robot 0 (Ego):** $0.0\text{ m}$ (Rear of formation)
- **Robot 1 (Opponent 1):** $+5.0\text{ m}$ ahead
- **Robot 2 (Opponent 2):** $+10.0\text{ m}$ ahead
- **Robot 3 (Opponent 3):** $+15.0\text{ m}$ ahead

### Dynamic Overtaking Results
- **Opponent 2 Overtaken:** Cleanly executed at **17.25 s** ($y = 56.16\text{ m}$)
- **Opponent 3 Overtaken:** Cleanly executed at **17.57 s** ($y = 57.03\text{ m}$)
- **Opponent 1 Overtaken:** Cleanly executed at **18.09 s** ($y = 58.69\text{ m}$)
- **Collision Count:** **0 collisions (100% clean passes)**.

---

## 📦 Challenge 3: ROS 2 Bag Recording & Replay

### 1. Recording Live Topics
To record all active topics into `f1tenth_run_01`:
```bash
./scripts/record_rosbag.sh
# Or directly:
ros2 bag record -a -o "ROS 2 Bag"
```
Press `Ctrl + C` when finished.

### 2. Inspecting Bag Metadata
```bash
ros2 bag info "ROS 2 Bag"
```

### 3. Playing Back Recorded Bag
```bash
./scripts/play_rosbag.sh 1.0
# Or at half speed:
./scripts/play_rosbag.sh 0.5
# Or directly:
ros2 bag play "ROS 2 Bag" --rate 0.5
```

---

## ⏱️ Lap Counter & Lap Timer Implementation
- **Spatial Hysteresis:** Start/finish line at $(0.0, 0.0)$. The trigger arms once $y > 15\text{ m}$ or $t > 15\text{ s}$, and triggers on transition from $y < 0$ to $y \ge 0$ with $|x| < 3.0\text{ m}$.
- **HUD & Telemetry GUI:** `telemetry_gui.py` displays real-time lap count, current split stopwatch, best lap time, speed gauge, and scrollable 10-lap history.
- **Automatic Parking Brake:** Upon completing Lap 10, an active 50 Hz brake timer publishes $0.0\text{ m/s}$ drive commands to park the vehicle safely.

---

## 📁 Repository File & Code Structure
```
.
├── README.md                                  # Complete GitHub Documentation & Tutorial
├── SUBMISSION.md                              # Official E-Campus / LMS Submission Document
├── sakhir_lap_records.csv                     # Official 10-lap benchmark record CSV
├── maps/
│   ├── Sakhir_map.png / .yaml                # Original clean Sakhir track map
│   └── Sakhir_obs_map.png / .yaml            # Modified map with 5 static obstacles
├── scripts/
│   ├── verify_static_10laps.py               # Headless verification script for Challenge 1
│   ├── verify_dynamic_overtaking.py          # Headless verification script for Challenge 2
│   ├── record_rosbag.sh                      # ROS 2 bag record script (Challenge 3)
│   ├── play_rosbag.sh                        # ROS 2 bag playback script (Challenge 3)
│   └── open_submission.sh                    # Submission launcher script
├── 'ROS 2 Bag'/                              # Official ROS 2 Bag Dataset (49,463 msgs)
│   ├── f1tenth_run_01_0.db3                  # SQLite3 message database
│   ├── metadata.yaml                         # Bag metadata specification
│   └── README.md                             # Dataset guide & playback instructions
├── docs/
│   └── ORAL_DEFENSE_PREP.md                  # Comprehensive Oral Defense Interview Guide
└── src/
    └── controllers/                          # ROS 2 Package
        ├── package.xml
        ├── setup.py
        ├── controllers/
        │   ├── gap_node.py                   # Reactive Follow-The-Gap ROS 2 Node
        │   └── telemetry_gui.py              # OpenCV Live Telemetry HUD Overlay Node
        └── launch/
            ├── competition_sakhir_static.launch.py   # Challenge 1 Launch File
            └── competition_sakhir_dynamic.launch.py  # Challenge 2 Launch File
```

---

## 🚀 Installation & Execution Guide

### 1. Build the Workspace
```bash
cd ~/sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select controllers
source install/setup.bash
```

### 2. Run Challenge 1 (Static Obstacles 10-Lap Verification)
```bash
python3 scripts/verify_static_10laps.py
```

### 3. Run Challenge 2 (Dynamic 4-Robot Overtaking Verification)
```bash
python3 scripts/verify_dynamic_overtaking.py
```

### 4. Launch Full ROS 2 Simulation in RViz
```bash
ros2 launch controllers competition_sakhir_static.launch.py
```

---

## 🎙️ Oral Defense Preparation Summary
A comprehensive study guide covering every theoretical and implementation question is available in [`docs/ORAL_DEFENSE_PREP.md`](docs/ORAL_DEFENSE_PREP.md). Key topics include:
- Follow-The-Gap mathematics & Gaussian smoothing rationale
- Safety bubble angle projection formula $\theta_{\text{bubble}} = \arctan(R / d_{\min})$
- Prevention of high-speed fishtailing via Speed-Scheduled Steering Gain Attenuation
- Spatial hysteresis lap counter & 50 Hz active parking brake
- GIMP map modification workflow per AV Class 8

---
*Developed by Islam Qaj Mazarul for the Autonomous Vehicles Course Final Project (Sakhir Circuit, Group 2).*
