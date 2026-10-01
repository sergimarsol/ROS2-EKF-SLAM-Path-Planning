#!/usr/bin/env python3
"""
Lawn mower waypoint generator (straight-line sweeps + in-place turns)

Generates ordered straight-line trajectory for a square arena (default 7ft x 7ft),
records the coordinates where in-place turns happen (endpoints of each sweep row and connectors)
and saves them to CSV as x,y (meters).
Also plots the full trajectory and saves a PNG.

Outputs are stored in:
  /coverage_navigation/coverage_navigation/output/turn_waypoints.csv
  /coverage_navigation/coverage_navigation/output/lawnmower_straight_trajectory.png
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# -------------------- USER PARAMETERS --------------------
ft2m = 0.3048
arena_ft = 7.0                 # arena side length in feet
arena_m = arena_ft * ft2m

robot_radius = 0.15            # robot radius in meters (for margin)
robot_width = 0.20             # robot width in meters
overlap_frac = 0.10            # fraction overlap between adjacent passes (0..1)
step_s = robot_width * (1.0 - overlap_frac)  # lateral spacing between sweep rows

margin = robot_radius + 0.05   # safety margin from walls (m)
point_spacing = 0.05           # spacing along sweeps for plotting/trajectory sampling (m)

# -------------------- OUTPUT PATHS --------------------
script_dir = os.path.dirname(os.path.abspath(__file__))
output_dir = os.path.join(script_dir, "output")
os.makedirs(output_dir, exist_ok=True)

out_csv = os.path.join(output_dir, "turn_waypoints.csv")
out_plot = os.path.join(output_dir, "lawnmower_straight_trajectory.png")
# ---------------------------------------------------------

def generate_sweep_rows(xmin, xmax, ymin, ymax, step_s):
    ys = []
    cur = ymin
    eps = 1e-9
    while cur <= ymax + eps:
        ys.append(round(cur, 9))
        cur += step_s
    if abs(ys[-1] - ymax) > 1e-6:
        ys.append(round(ymax, 9))
    return ys

def build_straight_trajectory(xmin, xmax, ys, point_spacing):
    trajectory = []
    turn_points = []

    start_x = xmin
    start_y = ys[0]
    trajectory.append((start_x, start_y))
    turn_points.append((start_x, start_y))  # Starting turn point

    for i, y in enumerate(ys):
        x_left = xmin
        x_right = xmax

        # Horizontal sweep
        if i % 2 == 0:
            xs = np.arange(x_left, x_right + point_spacing/2.0, point_spacing)
            for x in xs[1:]:
                trajectory.append((float(x), float(y)))
            end_pt = (float(x_right), float(y))
        else:
            xs = np.arange(x_right, x_left - point_spacing/2.0, -point_spacing)
            for x in xs[1:]:
                trajectory.append((float(x), float(y)))
            end_pt = (float(x_left), float(y))

        turn_points.append(end_pt)  # End of horizontal sweep

        # Vertical connector to next row
        if i < len(ys) - 1:
            next_y = ys[i+1]
            connector_x = end_pt[0]
            n_conn = max(2, int(abs(next_y - y) / point_spacing))
            for t in range(1, n_conn + 1):
                frac = t / (n_conn + 0.0)
                py = y + (next_y - y) * frac
                trajectory.append((connector_x, float(py)))
            # Add the top of the vertical connector as a turn
            turn_points.append((connector_x, next_y))

    return trajectory, turn_points

def save_turn_points_csv(turn_points, filename):
    df = pd.DataFrame(turn_points, columns=["x_m", "y_m"])
    df.index.name = "order"
    df.to_csv(filename, index=True)
    return df

def plot_trajectory(trajectory, turn_points, xmin_world, xmax_world, ymin_world, ymax_world,
                    xmin_eff, xmax_eff, ymin_eff, ymax_eff, robot_width, out_plot):
    traj = np.array(trajectory)
    plt.figure(figsize=(6,6))

    # Draw arena boundary
    plt.plot([xmin_world, xmax_world, xmax_world, xmin_world, xmin_world],
             [ymin_world, ymin_world, ymax_world, ymax_world, ymin_world], linestyle='-', linewidth=1, color='black')

    # Draw effective coverage rectangle
    plt.plot([xmin_eff, xmax_eff, xmax_eff, xmin_eff, xmin_eff],
             [ymin_eff, ymin_eff, ymax_eff, ymax_eff, ymin_eff], linestyle='--', linewidth=1, color='gray')

    # Draw coverage rectangles for each sweep row
    for i, (x, y) in enumerate(turn_points):
        rect_y = y - robot_width/2
        plt.fill_between([xmin_eff, xmax_eff], rect_y, rect_y + robot_width, color='cyan', alpha=0.3)

    # Draw trajectory
    if traj.size > 0:
        plt.plot(traj[:,0], traj[:,1], marker='o', linewidth=1, markersize=3, color='blue', label='Trajectory')

    # Draw turn points (only)
    if len(turn_points) > 0:
        tp = np.array(turn_points)
        plt.scatter(tp[:,0], tp[:,1], s=40, marker='D', color='red', label='Waypoints')

    plt.title("Lawnmower trajectory with coverage (robot width + overlap)")
    plt.axis('equal')
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.grid(True)
    plt.legend()
    plt.savefig(out_plot, dpi=150, bbox_inches='tight')
    plt.show()

def main():
    xmin_world = 0.0
    ymin_world = 0.0
    xmax_world = arena_m
    ymax_world = arena_m
    xmin = xmin_world + margin
    xmax = xmax_world - margin
    ymin = ymin_world + margin
    ymax = ymax_world - margin

    if xmin >= xmax or ymin >= ymax:
        raise ValueError("Margin too large for arena size. Reduce margin or change arena size.")

    ys = generate_sweep_rows(xmin, xmax, ymin, ymax, step_s)
    trajectory, turn_points = build_straight_trajectory(xmin, xmax, ys, point_spacing)
    df_turns = save_turn_points_csv(turn_points, out_csv)

    traj_arr = np.array(trajectory)
    if traj_arr.shape[0] >= 2:
        dists = np.sqrt(np.sum(np.diff(traj_arr, axis=0)**2, axis=1))
        path_length = float(np.sum(dists))
    else:
        path_length = 0.0

    print("=== Lawn mower waypoint generator ===")
    print(f"Arena: {arena_ft} ft x {arena_ft} ft -> {arena_m:.4f} m x {arena_m:.4f} m")
    print(f"Effective coverage rectangle after margin = {margin:.3f} m: x in [{xmin:.3f}, {xmax:.3f}], y in [{ymin:.3f}, {ymax:.3f}]")
    print(f"Robot width: {robot_width:.2f} m, lateral spacing s = {step_s:.3f} m with {overlap_frac*100:.1f}% overlap")
    print(f"Number of sweep rows: {len(ys)}")
    print(f"Saved in-place turn points: {len(df_turns)} -> '{out_csv}'")
    print(f"Estimated straight-line path length: {path_length:.3f} m")
    print()

    plot_trajectory(trajectory, turn_points, xmin_world, xmax_world, ymin_world, ymax_world,
                    xmin, xmax, ymin, ymax, robot_width, out_plot)
    print(f"Plot saved to: {out_plot}")

if __name__ == "__main__":
    main()
