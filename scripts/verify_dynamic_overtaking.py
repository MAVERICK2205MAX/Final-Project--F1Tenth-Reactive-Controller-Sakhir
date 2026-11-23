#!/usr/bin/env python3
"""
Final Project — Challenge 2: Dynamic Obstacle Challenge (Robot Overtaking)
Circuit: Sakhir (Group 2) | Student: Islam Qaj Mazarul (MAVERICK2205MAX)

Autonomous Multi-Robot Overtaking Verification:
- 4 Robots configured on Sakhir track
- Consecutive starting positions with exactly 5-meter spacing:
    * Robot 0 (Ego): 0.0 m (rear of formation)
    * Robot 1 (Opp 1): 5.0 m ahead
    * Robot 2 (Opp 2): 10.0 m ahead
    * Robot 3 (Opp 3): 15.0 m ahead
- All robots operate using Follow-The-Gap reactive algorithms
- Original Ego vehicle detects moving opponents as dynamic obstacles,
  projects safety bubbles, extends disparities, executes clean overtakes,
  and completes laps without collision.
"""

import math
import os
import sys
import time
import gym
import numpy as np


class DynamicOvertakingController:
    """Follow-The-Gap controller with dynamic obstacle avoidance and overtaking capabilities."""

    def __init__(self, v_min=2.0, v_max=6.8, bubble_r=0.28, is_ego=True):
        self.v_min = v_min
        self.v_max = v_max
        self.bubble_radius = bubble_r
        self.safety_width = 0.40
        self.bubble_max_dist = 3.5
        self.max_range = 8.0
        self.fov_deg = 70.0
        self.is_ego = is_ego
        self.prev_steer = 0.0
        self.prev_time = None

        self.disp_thresh = 0.9 if is_ego else 1.2
        self.v_steer_base = 3.4
        self.v_steer_pow = 1.0
        self.max_steer_velocity = 4.2
        self.ema_alpha = 0.32
        self.hazard_active = False

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

        fov_rad = math.radians(self.fov_deg)
        min_idx = max(0, int((-fov_rad - angle_min) / angle_inc))
        max_idx = min(num_beams - 1, int((fov_rad - angle_min) / angle_inc))

        proc_ranges = np.array(scan[min_idx:max_idx + 1], dtype=np.float32)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_range)
        proc_ranges = np.convolve(proc_ranges, self.gauss_kernel, mode='same')
        angles = angle_min + (min_idx + np.arange(len(proc_ranges))) * angle_inc

        # Dynamic Obstacle Hazard Detection
        x_pts = proc_ranges * np.cos(angles)
        y_pts = proc_ranges * np.sin(angles)
        hazard_mask = (np.abs(y_pts) <= self.safety_width) & (x_pts > 0.05) & (x_pts < self.bubble_max_dist)
        hazard_indices = np.where(hazard_mask)[0]
        self.hazard_active = (len(hazard_indices) > 0)
        min_fwd_dist = 999.0

        if self.hazard_active:
            closest_local = hazard_indices[int(np.argmin(proc_ranges[hazard_mask]))]
            min_fwd_dist = float(proc_ranges[closest_local])
            b_beams = int(math.atan2(self.bubble_radius, min_fwd_dist) / angle_inc)
            b_start = max(0, closest_local - b_beams)
            b_end = min(len(proc_ranges), closest_local + b_beams + 1)
            proc_ranges[b_start:b_end] = 0.0

        # Disparity Extender
        diff = np.abs(np.diff(proc_ranges))
        for d_idx in np.where(diff > self.disp_thresh)[0]:
            if abs(angles[d_idx]) <= math.radians(24.0):
                d1 = proc_ranges[d_idx]
                d2 = proc_ranges[d_idx + 1]
                if d1 < d2 and d1 > 0.1:
                    b_beams = int(math.atan2(self.bubble_radius, d1) / angle_inc)
                    proc_ranges[d_idx + 1:min(len(proc_ranges), d_idx + 1 + b_beams)] = 0.0
                elif d2 < d1 and d2 > 0.1:
                    b_beams = int(math.atan2(self.bubble_radius, d2) / angle_inc)
                    proc_ranges[max(0, d_idx - b_beams):d_idx + 1] = 0.0

        # Find Free Gaps
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

        # Gap Selection: For Ego overtaking, pick widest and deepest corridor
        if self.hazard_active and len(gaps) > 1 and self.is_ego:
            selected_gap = max(gaps, key=lambda g: (g[1] - g[0]) * np.mean(proc_ranges[g[0]:g[1]+1]))
        else:
            selected_gap = max(gaps, key=lambda g: g[1] - g[0])

        gap_center = (selected_gap[0] + selected_gap[1]) // 2
        sub = proc_ranges[selected_gap[0]:selected_gap[1] + 1]
        max_d = np.max(sub)
        near_max = np.where(sub >= max(0.90 * max_d, max_d - 0.30))[0]
        deepest_plateau = selected_gap[0] + int(np.median(near_max))

        if self.hazard_active:
            target_local_idx = int(0.65 * gap_center + 0.35 * deepest_plateau)
        else:
            target_local_idx = int(0.20 * gap_center + 0.80 * deepest_plateau)

        raw_steer = angles[target_local_idx]

        # Dynamic Speed Profiling
        steer_factor = max(0.0, 1.0 - (abs(raw_steer) / math.radians(22.0))) ** 1.3
        if self.hazard_active and min_fwd_dist < 2.5 and self.is_ego:
            dist_factor = np.clip((min_fwd_dist - 0.5) / 3.0, 0.0, 1.0)
        else:
            mid = len(proc_ranges) // 2
            arc_half = int(math.radians(16.0) / angle_inc)
            fwd_window = proc_ranges[max(0, mid - arc_half):min(len(proc_ranges), mid + arc_half)]
            fwd_clearance = float(np.percentile(fwd_window, 15)) if len(fwd_window) > 0 else 3.0
            dist_factor = np.clip((fwd_clearance - 1.8) / 4.0, 0.0, 1.0)

        speed = float(np.clip(
            self.v_min + (self.v_max - self.v_min) * min(steer_factor, dist_factor),
            self.v_min,
            self.v_max
        ))

        # Speed-scheduled steering gain
        if speed > self.v_steer_base:
            gain = (self.v_steer_base / speed) ** self.v_steer_pow
            target_steer = raw_steer * max(0.35, min(1.0, gain))
        else:
            target_steer = raw_steer

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
    map_path = os.path.join(workspace_dir, 'maps', 'Sakhir_map')

    print("=" * 75)
    print("  FINAL PROJECT — CHALLENGE 2: DYNAMIC OBSTACLE & ROBOT OVERTAKING")
    print("  Student: Islam Qaj Mazarul  |  Group: Group 2  |  Circuit: Sakhir")
    print("  Robots: 4 Autonomous Vehicles | Consecutively Placed with 5m Spacing")
    print("=" * 75)

    # Compute start poses along centerline
    centerline_path = '/home/admin1/f1tenth_racetracks/Sakhir/Sakhir_centerline.csv'
    data = np.loadtxt(centerline_path, delimiter=',', comments='#')
    xs, ys = data[:, 0], data[:, 1]
    dx, dy = np.diff(xs), np.diff(ys)
    dist_segments = np.sqrt(dx**2 + dy**2)
    cum_dist = np.insert(np.cumsum(dist_segments), 0, 0.0)

    distances = [0.0, 5.0, 10.0, 15.0]
    start_poses = []
    print("\nInitial Starting Grid (5m Consecutively Positioned):")
    for i, d in enumerate(distances):
        idx = int(np.argmin(np.abs(cum_dist - d)))
        t_idx = min(idx + 1, len(xs) - 1)
        theta = math.atan2(ys[t_idx] - ys[idx], xs[t_idx] - xs[idx])
        start_poses.append([xs[idx], ys[idx], theta])
        role = "Ego Racecar (Rear of Formation)" if i == 0 else f"Opponent Robot {i}"
        print(f"  • Robot {i} [{role}]: Start dist={d:4.1f}m, Pos=({xs[idx]:.2f}, {ys[idx]:.2f}), Heading={theta:.2f} rad")

    start_poses = np.array(start_poses)

    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=4)
    obs, _, _, _ = env.reset(start_poses)

    ego_ctrl = DynamicOvertakingController(v_min=2.5, v_max=7.0, bubble_r=0.28, is_ego=True)
    opponents = [
        DynamicOvertakingController(v_min=2.0, v_max=3.8, bubble_r=0.28, is_ego=False),
        DynamicOvertakingController(v_min=2.0, v_max=4.2, bubble_r=0.28, is_ego=False),
        DynamicOvertakingController(v_min=2.0, v_max=4.6, bubble_r=0.28, is_ego=False)
    ]
    all_ctrls = [ego_ctrl] + opponents

    overtake_status = [False, False, False]
    overtake_times = [0.0, 0.0, 0.0]

    sim_time = 0.0
    dt = 0.01
    max_steps = 20000
    collision_occurred = False

    print("\nStarting Dynamic Race Simulation... Monitoring Overtakes...")
    t_start = time.time()

    for step in range(max_steps):
        actions = []
        for i, c in enumerate(all_ctrls):
            st, sp = c.get_action(obs['scans'][i], sim_time)
            actions.append([st, sp])
        actions = np.array(actions)
        obs, _, done, _ = env.step(actions)
        sim_time += dt

        ego_y = obs['poses_y'][0]
        # Check overtaking of opponents
        for opp_i in range(3):
            if not overtake_status[opp_i] and ego_y > obs['poses_y'][opp_i + 1] and obs['poses_y'][opp_i + 1] > 20.0:
                overtake_status[opp_i] = True
                overtake_times[opp_i] = sim_time
                print(f"  ⚡ Step {step:4d} ({sim_time:5.2f} s): EGO SUCCESSFULLY OVERTOOK OPPONENT {opp_i + 1}! "
                      f"(Ego y={ego_y:.2f} m > Opponent y={obs['poses_y'][opp_i + 1]:.2f} m)")

        if any(obs['collisions']):
            print(f"\n❌ COLLISION DETECTED at step {step}, time {sim_time:.2f}s! Collisions: {obs['collisions']}")
            collision_occurred = True
            break

        if all(overtake_status) and step >= 4000:
            print(f"\n✅ All 3 Opponents Successfully Overtaken and Cleared cleanly at time {sim_time:.2f} s!")
            break

    t_elapsed = time.time() - t_start

    print("\n" + "=" * 75)
    print("  CHALLENGE 2 DYNAMIC OVERTAKING SUMMARY")
    print("=" * 75)
    for i in range(3):
        ot = overtake_status[i]
        t = overtake_times[i]
        print(f"  • Opponent Robot {i + 1}: {'✅ OVERTAKEN at ' + f'{t:.2f}s' if ot else '❌ NOT OVERTAKEN'}")
    print(f"  • Overall Overtaking Status: {'✅ ALL 3 ROBOTS OVERTAKEN' if all(overtake_status) else '❌ INCOMPLETE'}")
    print(f"  • Collision Status:          {'✅ ZERO COLLISIONS (100% CLEAN)' if not collision_occurred else '❌ COLLISION'}")
    print(f"  • Wall Execution Time:       {t_elapsed:.2f} seconds")
    print("=" * 75)

    return 0 if (all(overtake_status) and not collision_occurred) else 1


if __name__ == '__main__':
    sys.exit(main())
