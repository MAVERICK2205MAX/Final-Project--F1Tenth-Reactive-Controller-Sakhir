#!/usr/bin/env python3
"""
Final Project — Challenge 1: Static Obstacle Challenge Verification
Circuit: Sakhir (Group 2) | Student: Islam Qaj Mazarul (MAVERICK2205MAX)

Autonomous 10-Lap Verification with 5 Static Obstacles:
- Verifies 10 consecutive clean laps on Sakhir_obs_map
- Follow-The-Gap reactive algorithm with Hazard Cone Safety Bubble & Adaptive Gap Centering
- Zero collisions with track boundaries or any of the 5 obstacles
- Automatic Lap Counter & Split Timer
- Generates official benchmark record CSV
"""

import math
import os
import sys
import time
import gym
import numpy as np


class FollowTheGapStaticController:
    """Adaptive Follow-The-Gap Reactive Controller with Obstacle Evasion & Anti-Wobble Smoothing."""

    def __init__(self, bubble_radius=0.28, safety_width=0.38, bubble_max_dist=3.0,
                 max_range=8.0, fov_deg=70.0, v_min=2.5, v_max=7.8):
        self.bubble_radius = bubble_radius
        self.safety_width = safety_width
        self.bubble_max_dist = bubble_max_dist
        self.max_range = max_range
        self.fov_deg = fov_deg
        self.v_min = v_min
        self.v_max = v_max
        self.prev_steer = 0.0
        self.prev_time = None

        self.disp_thresh = 1.1
        self.v_steer_base = 3.6
        self.v_steer_pow = 1.0
        self.max_steer_velocity = 4.0
        self.d_floor = 1.8
        self.d_range = 4.0
        self.ema_alpha = 0.32
        self.hazard_active = False

        # 7-point Gaussian filter
        self.gauss_kernel = np.array([1, 3, 6, 7, 6, 3, 1], dtype=np.float32)
        self.gauss_kernel /= np.sum(self.gauss_kernel)

    def get_action(self, scan, current_time=None):
        num_beams = len(scan)
        angle_min = -2.356194490192345
        angle_inc = 4.71238898038469 / num_beams

        if current_time is None:
            dt = 0.01
        elif self.prev_time is None:
            dt = 0.01
            self.prev_time = current_time
        else:
            dt = max(0.001, min(0.05, current_time - self.prev_time))
            self.prev_time = current_time

        # 1. FOV restriction [-fov_deg, +fov_deg]
        fov_rad = math.radians(self.fov_deg)
        min_idx = max(0, int((-fov_rad - angle_min) / angle_inc))
        max_idx = min(num_beams - 1, int((fov_rad - angle_min) / angle_inc))

        # 2. Slicing, clamping, and Gaussian smoothing
        proc_ranges = np.array(scan[min_idx:max_idx + 1], dtype=np.float32)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_range)
        proc_ranges = np.convolve(proc_ranges, self.gauss_kernel, mode='same')
        angles = angle_min + (min_idx + np.arange(len(proc_ranges))) * angle_inc

        # 3. Cartesian Travel Corridor Hazard Detection
        x_pts = proc_ranges * np.cos(angles)
        y_pts = proc_ranges * np.sin(angles)
        hazard_mask = (np.abs(y_pts) <= self.safety_width) & (x_pts > 0.05) & (x_pts < self.bubble_max_dist)
        hazard_indices = np.where(hazard_mask)[0]
        self.hazard_active = (len(hazard_indices) > 0)

        if self.hazard_active:
            closest_local = hazard_indices[int(np.argmin(proc_ranges[hazard_mask]))]
            min_dist = float(proc_ranges[closest_local])
            b_beams = int(math.atan2(self.bubble_radius, min_dist) / angle_inc)
            b_start = max(0, closest_local - b_beams)
            b_end = min(len(proc_ranges), closest_local + b_beams + 1)
            proc_ranges[b_start:b_end] = 0.0

        # 4. Disparity Extender
        diff = np.abs(np.diff(proc_ranges))
        for d_idx in np.where(diff > self.disp_thresh)[0]:
            if abs(angles[d_idx]) <= math.radians(22.0):
                d1 = proc_ranges[d_idx]
                d2 = proc_ranges[d_idx + 1]
                if d1 < d2 and d1 > 0.1:
                    b_beams = int(math.atan2(self.bubble_radius, d1) / angle_inc)
                    proc_ranges[d_idx + 1:min(len(proc_ranges), d_idx + 1 + b_beams)] = 0.0
                elif d2 < d1 and d2 > 0.1:
                    b_beams = int(math.atan2(self.bubble_radius, d2) / angle_inc)
                    proc_ranges[max(0, d_idx - b_beams):d_idx + 1] = 0.0

        # 5. Find Free Gaps
        free_mask = proc_ranges > 0.7
        gaps = []
        in_gap = False
        start = 0
        for i, free in enumerate(free_mask):
            if free and not in_gap:
                in_gap = True
                start = i
            elif not free and in_gap:
                in_gap = False
                gaps.append((start, i - 1))
        if in_gap:
            gaps.append((start, len(proc_ranges) - 1))

        if not gaps:
            return 0.0, self.v_min

        max_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_center = (max_gap[0] + max_gap[1]) // 2
        sub = proc_ranges[max_gap[0]:max_gap[1] + 1]
        max_d = np.max(sub)
        near_max = np.where(sub >= max(0.90 * max_d, max_d - 0.30))[0]
        deepest_plateau = max_gap[0] + int(np.median(near_max))

        # Adaptive Gap Centering:
        # If obstacle detected in corridor: 60% center, 40% depth to avoid wall clipping.
        # Clear track: 20% center, 80% depth for high-speed racing line apex tracking.
        if self.hazard_active:
            target_local_idx = int(0.60 * gap_center + 0.40 * deepest_plateau)
        else:
            target_local_idx = int(0.20 * gap_center + 0.80 * deepest_plateau)

        raw_steer = angles[target_local_idx]

        # 6. Dynamic Curvature & Proximity Braking
        steer_factor = max(0.0, 1.0 - (abs(raw_steer) / math.radians(22.0))) ** 1.2
        mid = len(proc_ranges) // 2
        arc_half = int(math.radians(16.0) / angle_inc)
        fwd_window = proc_ranges[max(0, mid - arc_half):min(len(proc_ranges), mid + arc_half)]
        fwd_clearance = float(np.percentile(fwd_window, 15)) if len(fwd_window) > 0 else 3.0
        dist_factor = np.clip((fwd_clearance - self.d_floor) / self.d_range, 0.0, 1.0) ** 1.0

        speed = float(np.clip(
            self.v_min + (self.v_max - self.v_min) * min(steer_factor, dist_factor),
            self.v_min,
            self.v_max
        ))

        # 7. Speed-Scheduled Steering Gain Attenuation (Prevents High-Speed Wobble)
        if speed > self.v_steer_base:
            gain = (self.v_steer_base / speed) ** self.v_steer_pow
            target_steer = raw_steer * max(0.32, min(1.0, gain))
        else:
            target_steer = raw_steer

        # 8. Exponential Moving Average & Slew Rate Limiter
        filtered_steer = (1.0 - self.ema_alpha) * self.prev_steer + self.ema_alpha * target_steer
        max_d_steer = self.max_steer_velocity * dt
        steer_angle = float(np.clip(
            filtered_steer,
            self.prev_steer - max_d_steer,
            self.prev_steer + max_d_steer
        ))
        self.prev_steer = steer_angle
        return steer_angle, speed


def main():
    workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    map_path = os.path.join(workspace_dir, 'maps', 'Sakhir_obs_map')

    print("=" * 75)
    print("  FINAL PROJECT — CHALLENGE 1: STATIC OBSTACLES (10 LAPS VERIFICATION)")
    print("  Student: Islam Qaj Mazarul  |  Group: Group 2  |  Circuit: Sakhir")
    print("  Obstacles: 5 Static Pylons Added per Class 8 Guidelines")
    print("=" * 75)

    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1)
    obs, _, _, _ = env.reset(np.array([[0.0, 0.0, 1.5252]]))
    controller = FollowTheGapStaticController()

    target_laps = 10
    lap_count = 0
    lap_times = []
    lap_start_sim_time = 0.0
    sim_time = 0.0
    dt = 0.01
    prev_y = 0.0
    armed = False
    collision_occurred = False

    t_real_start = time.time()
    max_steps = 150000

    print("\nStarting autonomous race simulation around Sakhir with 5 static obstacles...")

    for step in range(max_steps):
        steer, speed = controller.get_action(obs['scans'][0], sim_time)
        obs, _, done, _ = env.step(np.array([[steer, speed]]))
        sim_time += dt
        x, y = obs['poses_x'][0], obs['poses_y'][0]

        # Spatial hysteresis arming
        if y > 15.0 or x > 15.0 or (sim_time - lap_start_sim_time) > 15.0:
            armed = True

        # Finish line crossing
        if armed and prev_y < 0.0 and y >= 0.0 and abs(x) < 3.0:
            lap_time = sim_time - lap_start_sim_time
            lap_count += 1
            lap_times.append(lap_time)
            print(f"  🏁 Lap {lap_count:2d} / {target_laps} COMPLETED! "
                  f"Lap Time: {lap_time:6.2f} s | Total Elapsed: {sim_time:6.2f} s")
            lap_start_sim_time = sim_time
            armed = False
            if lap_count >= target_laps:
                break

        prev_y = y

        if obs['collisions'][0] > 0:
            print(f"\n❌ COLLISION DETECTED at step {step}, time {sim_time:.2f} s at ({x:.2f}, {y:.2f})!")
            collision_occurred = True
            break

    t_real_elapsed = time.time() - t_real_start

    print("\n" + "=" * 75)
    print("  BENCHMARK RESULTS & VERIFICATION")
    print("=" * 75)
    print(f"  Completed Laps:     {lap_count} / {target_laps}")
    print(f"  Collision Status:   {'✅ ZERO COLLISIONS (100% CLEAN)' if not collision_occurred else '❌ COLLISION OCCURRED'}")
    print(f"  Simulation Wall Time: {t_real_elapsed:.2f} seconds")
    print("-" * 75)
    print("   Lap #  |   Lap Time (s)   |   Status")
    print("-" * 75)

    best_lap = min(lap_times) if lap_times else 0.0
    avg_lap = sum(lap_times) / len(lap_times) if lap_times else 0.0

    csv_path = os.path.join(workspace_dir, 'sakhir_lap_records.csv')
    with open(csv_path, 'w') as f:
        f.write("Lap_Number,Lap_Time_s,Best_Lap_s,Total_Time_s,Obstacles,Status,Date_Time\n")
        cum_time = 0.0
        cur_best = float('inf')
        for i, t in enumerate(lap_times, 1):
            cum_time += t
            if t < cur_best:
                cur_best = t
            star = "  ⭐ FASTEST LAP!" if t == best_lap else ""
            print(f"   Lap {i:2d} |      {t:6.2f} s    |   Clean & Collision-Free{star}")
            f.write(f"{i},{t:.3f},{cur_best:.3f},{cum_time:.3f},5_Static_Obstacles,Clean,{time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    print("-" * 75)
    print(f"  🏆 FASTEST LAP TIME: {best_lap:.2f} seconds")
    print(f"  ⏱️  AVERAGE LAP TIME: {avg_lap:.2f} seconds")
    print(f"  📁 Official CSV written to: {csv_path}")
    print("=" * 75)

    return 0 if (lap_count >= target_laps and not collision_occurred) else 1


if __name__ == '__main__':
    sys.exit(main())
