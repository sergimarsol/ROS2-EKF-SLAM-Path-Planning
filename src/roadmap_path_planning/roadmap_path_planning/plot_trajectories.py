import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# Workspace and obstacle parameters
SQUARE_SIZE = 8.0  # feet
OBSTACLE_CENTER = np.array([4.0, 4.0])
OBSTACLE_SIZE = 1.0  # feet
OBSTACLE_MIN = OBSTACLE_CENTER - OBSTACLE_SIZE / 2

# Helper to load trajectory from CSV
def load_trajectory(csv_path):
    xs, ys = [], []
    with open(csv_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return np.array(xs), np.array(ys)

# Load both trajectories
csv_max_safety = os.path.join(os.path.dirname(__file__), '../output/real_trajectory_max_safety.csv')
csv_min_distance = os.path.join(os.path.dirname(__file__), '../output/real_trajectory_min_distance.csv')

xs_max, ys_max = load_trajectory(csv_max_safety)
xs_min, ys_min = load_trajectory(csv_min_distance)

fig, ax = plt.subplots(figsize=(8,8))
# Plot outer square (workspace)
ax.plot([0, SQUARE_SIZE, SQUARE_SIZE, 0, 0], [0, 0, SQUARE_SIZE, SQUARE_SIZE, 0], 'k-', linewidth=2)
# Plot obstacle (center square)
rect = plt.Rectangle(OBSTACLE_MIN, OBSTACLE_SIZE, OBSTACLE_SIZE, color='red', alpha=0.3)
ax.add_patch(rect)
# Plot trajectories
ax.plot(xs_max, ys_max, color='lime', linewidth=2, label='Max Safety Trajectory')
ax.plot(xs_min, ys_min, color='magenta', linewidth=2, label='Min Distance Trajectory')
# Mark start and end points with stars
ax.scatter([xs_max[0], xs_min[0]], [ys_max[0], ys_min[0]], color='green', marker='*', s=120, label='Start')
ax.scatter([xs_max[-1], xs_min[-1]], [ys_max[-1], ys_min[-1]], color='orange', marker='*', s=120, label='End')
# Plot ideal end point
ideal_x, ideal_y = 7.5, 7.5
ax.scatter([ideal_x], [ideal_y], color='blue', marker='*', s=160, label='Ideal End (7.5, 7.5)')
# Compute and print errors
err_max = np.hypot(xs_max[-1] - ideal_x, ys_max[-1] - ideal_y)
err_min = np.hypot(xs_min[-1] - ideal_x, ys_min[-1] - ideal_y)
print(f"Max Safety Trajectory End Error: {err_max:.3f} ft")
print(f"Min Distance Trajectory End Error: {err_min:.3f} ft")
# Axes and grid
ax.set_xlim(0, SQUARE_SIZE)
ax.set_ylim(0, SQUARE_SIZE)
ax.set_xlabel('x (feet)')
ax.set_ylabel('y (feet)')
ax.set_aspect('equal')
ax.set_title('Real Robot Trajectories')
ax.grid(True)
ax.legend()
output_dir = os.path.join(os.path.dirname(__file__), '../output')
os.makedirs(output_dir, exist_ok=True)
image_path = os.path.join(output_dir, 'final_trajectories.png')
plt.savefig(image_path)
print(f"Plot saved to {image_path}")
plt.show()
