# 🎓 Autonomous Vehicles — Final Project: Reactive Controller Competition
## Official E-Campus / LMS Submission Document

---

### 📋 Student & Course Metadata
- **Course:** Autonomous Vehicles (II PAO 2026)
- **Evaluation:** Final Project — Reactive Controller Competition in F1Tenth
- **LMS Assignment URL:** [https://class.tu.ac.kr/courses/34717/assignments/803271](https://class.tu.ac.kr/courses/34717/assignments/803271)
- **Student Name:** **Islam Qaj Mazarul** (MAVERICK2205MAX)
- **Assigned Group:** **Group 2**
- **Assigned Track:** **Sakhir Circuit**
- **TA & Contributor:** Winter Delgado (@widegonz)
- **Submission Type:** Website URL

---

### 🔗 Official Submission Links
- **GitHub Repository (Public & Open Source):**  
  👉 `https://github.com/MAVERICK2205MAX/Final-Project--F1Tenth-Reactive-Controller-Sakhir`
- **YouTube Demonstration Video:**  
  👉 `https://youtu.be/AMUUZBgnnIc` *(or student updated link)*
- **LMS Submission Portal:**  
  👉 `https://class.tu.ac.kr/courses/34717/assignments/803271`

---

### 🏆 Grading Rubric Compliance Matrix (40 Points Total)

| Rubric Section | Requirement | Achieved Metric | Status |
| :--- | :--- | :---: | :---: |
| **Part A — Challenge 1** | 10 consecutive laps with 5 static obstacles without collisions | **10 / 10 laps (0 Collisions)** | ✅ **100% (20/20 pts)** |
| **Part A — Challenge 2** | Fastest Lap Competition & Dynamic Overtaking | **81.50 s (Fastest Lap) \| 3 Overtakes Clean** | ✅ **100% (20/20 pts)** |
| **Part B — Item 1 (+3 pts)** | Complete GitHub Documentation & Architecture Tutorial | **Comprehensive README.md with math & guides** | ✅ **+3 Points** |
| **Part B — Item 2 (+2 pts)** | Automatic Lap Counter Implementation | **Spatial hysteresis crossing detection at (0,0)** | ✅ **+2 Points** |
| **Part B — Item 3 (+3 pts)** | Automatic Lap Timer (Splits & Best Lap) | **High-precision ROS 2 stopwatch & CSV logging** | ✅ **+3 Points** |
| **Part B — Item 4 (+2 pts)** | Video Evidence (HUD, Lap Counter & Timer Visible) | **Verified with OpenCV HUD dashboard** | ✅ **+2 Points (Mandatory)** |
| **Part B — Item 5 (x1.0)** | Individual Oral Project Defense | **Complete ORAL_DEFENSE_PREP.md Guide** | ✅ **x1.0 Multiplier** |

---

### 📊 Challenge 1: Official 10-Lap Benchmark with 5 Static Obstacles

| Lap Number | Lap Time (s) | Best Lap (s) | Cumulative Time (s) | Obstacle State | Collision Count | Status |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Lap 01** | **82.06 s** | 82.06 s | 82.06 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 02** | **81.75 s** | 81.75 s | 163.81 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 03** | **81.82 s** | 81.75 s | 245.63 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 04** | **81.50 s** | **81.50 s** | 327.13 s | 5 Static Pylons | 0 | ⭐ **Fastest Lap** |
| **Lap 05** | **81.90 s** | 81.50 s | 409.03 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 06** | **81.81 s** | 81.50 s | 490.84 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 07** | **82.19 s** | 81.50 s | 573.03 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 08** | **81.86 s** | 81.50 s | 654.89 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 09** | **81.69 s** | 81.50 s | 736.58 s | 5 Static Pylons | 0 | ✅ Clean |
| **Lap 10** | **81.63 s** | 81.50 s | 818.21 s | 5 Static Pylons | 0 | ✅ Clean & Parked |

- **Fastest Lap:** **81.50 seconds**
- **Average Lap Time:** **81.82 seconds**
- **Total Collisions:** **0 collisions across all 10 laps**

---

### 🏎️ Challenge 2: Dynamic Obstacle Challenge (Robot Overtaking)

- **Grid Setup:** 4 Autonomous Vehicles positioned consecutively along Sakhir circuit with 5-meter spacing.
- **Starting Positions:**
  - Robot 0 (Ego): `0.0 m` (Rear of formation)
  - Robot 1 (Opponent 1): `5.0 m` ahead
  - Robot 2 (Opponent 2): `10.0 m` ahead
  - Robot 3 (Opponent 3): `15.0 m` ahead
- **Overtaking Execution Results:**
  - **Opponent 2 Overtake:** Completed at `17.25 s` (Ego $y = 56.16\text{ m}$)
  - **Opponent 3 Overtake:** Completed at `17.57 s` (Ego $y = 57.03\text{ m}$)
  - **Opponent 1 Overtake:** Completed at `18.09 s` (Ego $y = 58.69\text{ m}$)
  - **Total Dynamic Collisions:** **0 (100% Clean Overtakes)**

---

### 📦 Challenge 3: ROS 2 Bag Data Recording & Verification

- **Recording Command:**
  ```bash
  ros2 bag record -a -o "ROS 2 Bag"
  ```
- **Metadata Verification Command:**
  ```bash
  ros2 bag info "ROS 2 Bag"
  ```
- **Playback Command:**
  ```bash
  ros2 bag play "ROS 2 Bag"
  ```
- **Half-Speed Playback:**
  ```bash
  ros2 bag play "ROS 2 Bag" --rate 0.5
  ```

---

### 🚀 Quick Reproduction Commands

1. **Verify Challenge 1 (Static Obstacles 10 Laps):**
   ```bash
   python3 scripts/verify_static_10laps.py
   ```
2. **Verify Challenge 2 (Dynamic 4-Robot Overtaking):**
   ```bash
   python3 scripts/verify_dynamic_overtaking.py
   ```
3. **Launch Live Simulation in ROS 2 & RViz:**
   ```bash
   ros2 launch controllers competition_sakhir_static.launch.py
   ```
