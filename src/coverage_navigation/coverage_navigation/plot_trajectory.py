import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# Workspace parameters (converted to meters)
SQUARE_SIZE = 7.0 * 0.3048  # 7 ft -> meters (~2.1336 m)

# Path to the trajectory file
csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'output', 'trajectory.csv')

# Output directory
output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'output')
os.makedirs(output_dir, exist_ok=True)

# Helper to load trajectory from CSV
def load_trajectory(csv_path):
    xs, ys = [], []
    with open(csv_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return np.array(xs), np.array(ys)

# Load trajectory
xs, ys = load_trajectory(csv_path)

# Plot
fig, ax = plt.subplots(figsize=(8,8))

# Plot outer square in meters
ax.plot([0, SQUARE_SIZE, SQUARE_SIZE, 0, 0],
        [0, 0, SQUARE_SIZE, SQUARE_SIZE, 0],
        'k-', linewidth=2)

# Plot trajectory
ax.plot(xs, ys, color='blue', linewidth=2, label='Robot Trajectory')

# Mark start and end
ax.scatter([xs[0]], [ys[0]], color='green', marker='*', s=120, label='Start')
ax.scatter([xs[-1]], [ys[-1]], color='red', marker='*', s=120, label='End')

# Axes and grid
ax.set_xlim(0, SQUARE_SIZE)
ax.set_ylim(0, SQUARE_SIZE)
ax.set_xlabel('x (meters)')
ax.set_ylabel('y (meters)')
ax.set_aspect('equal')
ax.set_title('Trajectory Plot (7×7 ft Workspace in Meters)')
ax.grid(True)
ax.legend()

# Save plot safely
image_path = os.path.join(output_dir, "trajectory_plot_meters.png")
try:
    plt.savefig(image_path, bbox_inches='tight', dpi=300)
    print(f"Plot successfully saved to {image_path}")
except Exception as e:
    print(f"Error saving plot: {e}")

plt.show()
