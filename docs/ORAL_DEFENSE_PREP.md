# 🎙️ F1Tenth Final Project: Individual Oral Defense Preparation Guide
### Student: Islam Qaj Mazarul | Group 2 | Track: Sakhir
### Instructors: Prof. Nabih Pico, Ph.D. & TA Winter Delgado (@widegonz)

---

## 📌 Defense Purpose & Grading Impact
The oral defense verifies the authenticity of your implementation and technical comprehension.
- **Thorough Understanding & Correct Answers:** **$\times 1.0$ Weighting Factor (Preserves 100% of your grade)**.
- **Significant Knowledge Gaps:** $\times < 1.0$ Weighting Factor (Reduces final grade proportionally).

---

## 🧠 Core Technical Questions & Perfect Defense Answers

### Q1: What is the reactive Follow-The-Gap (FTG) algorithm, and how does it work?
**Answer:**  
"Follow-The-Gap is a purely reactive local navigation algorithm that computes steering commands directly from raw 2D LiDAR range data without requiring global SLAM or pre-recorded waypoints. Its execution pipeline has 5 main stages:
1. **Field-of-View (FOV) Restriction & Preprocessing:** Slices the 1080-beam 270° LiDAR scan down to the vehicle's forward arc ($\pm 70^\circ$). We filter out NaNs and infinities, clamp ranges to 8.0 meters, and apply a 7-point Gaussian convolution filter to eliminate single-beam sensor noise.
2. **Hazard Detection & Safety Bubble:** Detects obstacles within the vehicle's travel corridor ($|y| \le 0.38\text{ m}$, $x < 3.0\text{ m}$). Around the nearest obstacle at distance $d_{\min}$, we calculate the angular bubble radius $\theta_{\text{bubble}} = \arctan(r_{\text{safe}} / d_{\min})$ and zero out all beams within that cone.
3. **Disparity Extender:** Detects sharp range differences ($\Delta r > 1.1\text{ m}$) where open space meets obstacle edges. We extend the bubble onto the farther side to prevent the car from attempting to squeeze past an edge that would clip the rear wheels.
4. **Gap Identification & Adaptive Centering:** Scans for contiguous beams above the free threshold ($r > 0.7\text{ m}$) and selects the largest viable gap. We then pick a target steering index using **Adaptive Gap Centering**:
   - When an obstacle is detected in our path, we use **60% gap center + 40% deepest plateau** to safely split the corridor between the obstacle and outer wall.
   - On a clear straight or corner, we use **20% gap center + 80% deepest plateau** to trace the optimal racing apex.
5. **Speed-Scheduled Steering & Dynamic Braking:** Calculates physical steering angle with an Exponential Moving Average (EMA) and slew rate limiter, and attenuates high-speed steering gain to prevent fishtailing."

---

### Q2: How did you implement the 5 static obstacles on the Sakhir map (Challenge 1)?
**Answer:**  
"Following the methodology taught in **AV - Class 8** by Prof. Nabih Pico and TA Winter Delgado:
1. We opened the official map `Sakhir_map.png` (2000×2000 resolution, $0.05695\text{ m/pixel}$, origin $[-26.17, -46.60]$).
2. In ROS 2 occupancy grids, pixel value `255` represents free drivable space, and `0` (black) represents an obstacle/wall.
3. We identified 5 long straightaway sectors using track curvature analysis ($\kappa < 0.008$) so obstacles represent realistic race pylons without blocking corners:
   - **Obstacle 1 (idx 75):** Pit straight exit ($x=0.99, y=30.65$)
   - **Obstacle 2 (idx 250):** North straight ($x=42.16, y=52.96$)
   - **Obstacle 3 (idx 490):** Infield straight ($x=23.38, y=25.21$)
   - **Obstacle 4 (idx 630):** Back straight ($x=16.24, y=12.18$)
   - **Obstacle 5 (idx 915):** South connector straight ($x=27.93, y=-24.77$)
4. Each obstacle was painted as a circular pylon of radius 3 pixels ($\approx 0.17\text{ m}$ radius, $0.34\text{ m}$ diameter) with a $0.40\text{ m}$ lateral offset.
5. This leaves $\approx 1.5\text{ m}$ of open drivable track on the opposite side (track width is $2.2\text{ m}$), enabling our Follow-The-Gap controller to detect the pylon, project a bubble, and smoothly overtake without hitting either the pylon or the outer wall."

---

### Q3: How did you implement and verify Challenge 2 (Dynamic Obstacle Challenge / Robot Overtaking)?
**Answer:**  
"For Challenge 2:
1. We instantiated the multi-agent simulator environment with `num_agents = 4`.
2. We placed the 4 robots consecutively along the Sakhir centerline with exact 5-meter spacing:
   - **Robot 0 (Ego):** $0.0\text{ m}$ (starting at the rear of the grid)
   - **Robot 1 (Opponent 1):** $+5.0\text{ m}$ ahead
   - **Robot 2 (Opponent 2):** $+10.0\text{ m}$ ahead
   - **Robot 3 (Opponent 3):** $+15.0\text{ m}$ ahead
3. All vehicles run autonomous Follow-The-Gap reactive controllers. The opponents run at paced racing speeds ($3.8\text{ to }4.6\text{ m/s}$), while Ego operates at full competitive speed ($7.0\text{ m/s}$).
4. As Ego closes in from behind, its forward LiDAR scans the opponent vehicle's chassis as a dynamic obstacle.
5. Our controller avoids the 'symmetric gap trap' by evaluating total gap integral depth, selecting the clear passing corridor, reducing speed slightly to maintain lateral grip, and accelerating through the open corridor.
6. **Result:** Ego cleanly overtook Opponent 2 at $17.25\text{ s}$, Opponent 3 at $17.57\text{ s}$, and Opponent 1 at $18.09\text{ s}$, completing laps with **zero collisions** across all 4 vehicles."

---

### Q4: How does your Lap Counter and Lap Timer work?
**Answer:**  
"We implemented a high-reliability spatial hysteresis trigger:
1. Sakhir's start/finish line is located at $(0.0, 0.0)$ with the track heading North ($+y$).
2. **Arming Phase:** When the car reaches $y > 15.0\text{ m}$, $x > 15.0\text{ m}$, or after $15\text{ s}$ of elapsed lap time, `armed` is set to `True`. This prevents accidental double-triggers at the start grid.
3. **Trigger Phase:** As the vehicle completes the lap traveling from $y < 0$ to $y \ge 0$ with $|x| < 3.0\text{ m}$, the lap counter increments (`lap_count += 1`).
4. **Timer:** The lap timer computes $\Delta t = t_{\text{sim}} - t_{\text{lap\_start}}$, updates the session best lap time, prints a celebratory ASCII banner to terminal, updates the live OpenCV HUD overlay, and appends the split to `sakhir_lap_records.csv`.
5. **Mission Stop:** Upon reaching Lap 10, an active 50 Hz parking brake timer is triggered, publishing `AckermannDriveStamped(speed=0.0)` to safely park the car on the main straight."

---

### Q5: How do you record and replay ROS 2 bag files (Challenge 3)?
**Answer:**  
"We use the standard ROS 2 command-line tools:
1. **Record all topics:**
   ```bash
   ros2 bag record -a -o "ROS 2 Bag"
   ```
   This captures `/scan`, `/ego_racecar/odom`, `/drive`, `/tf`, `/tf_static`, and `/map`.
2. **Inspect recording metadata:**
   ```bash
   ros2 bag info "ROS 2 Bag"
   ```
   Displays duration, storage format (sqlite3), message counts, and serialized topics.
3. **Playback:**
   ```bash
   ros2 bag play "ROS 2 Bag"
   # Replay at half speed:
   ros2 bag play "ROS 2 Bag" --rate 0.5
   ```"

---

### Q6: What causes high-speed steering oscillation (fishtailing/wobble) in Follow-The-Gap, and how did you solve it?
**Answer:**  
"High-speed fishtailing is caused by three compounding physical phenomena:
1. **LiDAR Quantization Noise:** Discrete beams jumping between wall features. We solved this with a 7-point Gaussian kernel.
2. **Parallel Wall Bubble False-Triggers:** When parallel side walls at $\pm 70^\circ$ enter the safety bubble, they alternatingly zero out left and right gaps. We solved this with a **Cartesian forward travel corridor check** ($|y| \le \text{safety\_width}$, $x \in [0.05, 3.0]\text{ m}$).
3. **Speed-Induced Lateral Oversteer:** Turning the wheels by the same angle at $7.5\text{ m/s}$ produces far higher lateral acceleration than at $2.5\text{ m/s}$. We implemented **Speed-Scheduled Steering Gain Attenuation**:
   $$\text{Gain} = \left(\frac{v_{\text{base}}}{v}\right)^p$$
   When traveling fast, target steering is scaled down, and smoothed through an Exponential Moving Average (EMA, $\alpha = 0.32$) and a physical slew rate limiter ($4.0\text{ rad/s}$). This completely eliminates oscillation while keeping the car ultra-responsive."
