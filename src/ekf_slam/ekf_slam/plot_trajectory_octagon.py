import os
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd

# Load trajectory from CSV
oct_traj_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'output', 'trajectory_octagon.csv')
oct_tags_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'output', 'trajectory_octagon_tags.csv')
oct_plot_path = os.path.join(os.path.dirname(oct_traj_path), 'trajectory_octagon_plot.png')

# Load trajectory
traj = []
with open(oct_traj_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        x = float(row['x'])
        y = float(row['y'])
        theta_deg = float(row['theta_deg'])
        traj.append((x, y, theta_deg))
traj = np.array(traj)

# Load tags
tags_df = pd.read_csv(oct_tags_path)

plt.figure(figsize=(8,8))
# Draw the trajectory as a line and points (showing density)
plt.plot(traj[:,0], traj[:,1], '-', color='blue', label='Trajectory')
plt.plot(traj[:,0], traj[:,1], 'o', color='blue', markersize=2, alpha=0.7)
plt.plot(traj[0,0], traj[0,1], 'go', markersize=10, label='Start')
plt.plot(traj[-1,0], traj[-1,1], 'ro', markersize=10, label='End')

# Draw the ideal octagon trajectory as dashed black line
ideal_oct_x = []
ideal_oct_y = []
num_sides = 8
side_length = 0.5
start_x, start_y, start_theta = 0.0, 0.0, 90.0
x, y, theta = start_x, start_y, start_theta
for i in range(num_sides+1):  # +1 to close the octagon
    ideal_oct_x.append(x)
    ideal_oct_y.append(y)
    theta_rad = np.deg2rad(theta)
    dx = side_length * np.cos(theta_rad)
    dy = side_length * np.sin(theta_rad)
    x += dx
    y += dy
    theta = (theta + 360/num_sides) % 360
plt.plot(ideal_oct_x, ideal_oct_y, 'k--', linewidth=2, label="Ideal Octagon")

# Plot tags (noisy/estimated)
for i, row in tags_df.iterrows():
    if i == 0:
        plt.plot(row['x'], row['y'], 'bs', markersize=8, label='Predicted Tag')
    else:
        plt.plot(row['x'], row['y'], 'bs', markersize=8)
    plt.text(row['x'] + 0.03, row['y'] + 0.03, str(int(row['tag_id'])), fontsize=12, color='blue', weight='bold')

# --- Plot theoretical AprilTag positions for the octagon ---
ft_to_m = 0.3048
inch_to_m = 0.0254
square_size = 7 * ft_to_m
center_x, center_y = -0.5, 0.5
half = square_size / 2
sep = 27 * inch_to_m
half_sep = sep / 2
sep_0_1 = 18 * inch_to_m
half_sep_0_1 = sep_0_1 / 2

theoretical_tag_positions = []
# East side
x_east = center_x + half
y_east1 = center_y - half + half_sep
y_east2 = center_y + half - half_sep
theoretical_tag_positions.append((6, x_east, y_east1))
theoretical_tag_positions.append((7, x_east, y_east2))
# North side
x_north1 = center_x + half - half_sep_0_1
x_north2 = center_x - half + half_sep_0_1
y_north = center_y + half
theoretical_tag_positions.append((0, x_north1, y_north))
theoretical_tag_positions.append((1, x_north2, y_north))
# West side
x_west = center_x - half
y_west1 = center_y + half - half_sep
y_west2 = center_y - half + half_sep
theoretical_tag_positions.append((2, x_west, y_west1))
theoretical_tag_positions.append((3, x_west, y_west2))
# South side
x_south1 = center_x - half + half_sep
x_south2 = center_x + half - half_sep
y_south = center_y - half
theoretical_tag_positions.append((4, x_south1, y_south))
theoretical_tag_positions.append((5, x_south2, y_south))

for tag_id, x, y in theoretical_tag_positions:
    plt.plot(x, y, 'ms', markersize=8, markerfacecolor='none', markeredgewidth=2, label='Theoretical Tag' if tag_id==0 else None)
    plt.text(x + 0.03, y - 0.05, str(tag_id), fontsize=12, color='magenta', weight='bold')

# --- Compute and plot errors for tags ---
for i, (tag_id, x_theo, y_theo) in enumerate(theoretical_tag_positions):
    # Find predicted tag with same id
    pred_row = tags_df[tags_df['tag_id'] == tag_id].iloc[0]
    x_pred, y_pred = pred_row['x'], pred_row['y']
    error = np.hypot(x_pred - x_theo, y_pred - y_theo)
    # Draw line between predicted and theoretical
    plt.plot([x_pred, x_theo], [y_pred, y_theo], 'r--', linewidth=1)
    # Annotate error
    plt.text((x_pred + x_theo)/2, (y_pred + y_theo)/2, f"{error:.2f}m", color='red', fontsize=9, ha='center', va='center', bbox=dict(facecolor='white', alpha=0.6, edgecolor='none', boxstyle='round,pad=0.1'))

# --- Compute and plot error for robot end position ---
# Theoretical end is (-0.35, -0.35), actual is traj[-1,0], traj[-1,1]
robot_end_x, robot_end_y = traj[-1,0], traj[-1,1]
ideal_end_x, ideal_end_y = -0.35, -0.35
robot_end_error = np.hypot(robot_end_x - ideal_end_x, robot_end_y - ideal_end_y)
plt.plot([robot_end_x, ideal_end_x], [robot_end_y, ideal_end_y], 'g--', linewidth=2)
plt.text((robot_end_x + ideal_end_x)/2, (robot_end_y + ideal_end_y)/2, f"{robot_end_error:.2f}m", color='green', fontsize=11, ha='center', va='center', bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', boxstyle='round,pad=0.1'))

plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Robot Octagon Trajectory and AprilTag Positions')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()

# Save the figure to the output directory
plt.savefig(oct_plot_path)
print(f"Trajectory and tags plot saved to {oct_plot_path}")
plt.show()
