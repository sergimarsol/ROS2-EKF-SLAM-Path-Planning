import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import csv
import os

# Constants
SQUARE_SIZE = 8.0  # feet
NUM_SAMPLES = 2000
OBSTACLE_CENTER = np.array([4.0, 4.0])
OBSTACLE_SIZE = 1.0  # feet
OBSTACLE_MIN = OBSTACLE_CENTER - OBSTACLE_SIZE / 2
OBSTACLE_MAX = OBSTACLE_CENTER + OBSTACLE_SIZE / 2
K_NEIGHBORS = 8
START = np.array([0.5, 0.5])
GOAL = np.array([7.5, 7.5])
CLEARANCE_THRESHOLD = 0.1  # meters (10cm)

class RoadmapNode:
    def __init__(self, pos, idx):
        self.pos = np.array(pos)
        self.idx = idx
        self.neighbors = []  # list of (neighbor_idx, length, clearance)

class Roadmap:
    def __init__(self):
        self.nodes = []
        self.edges = []  # list of (idx1, idx2, length, clearance)

    def add_node(self, pos):
        idx = len(self.nodes)
        node = RoadmapNode(pos, idx)
        self.nodes.append(node)
        return idx

    def add_edge(self, idx1, idx2, length, clearance):
        self.nodes[idx1].neighbors.append((idx2, length, clearance))
        self.nodes[idx2].neighbors.append((idx1, length, clearance))
        self.edges.append((idx1, idx2, length, clearance))

# Utility functions
def is_in_obstacle(point):
    return np.all(point >= OBSTACLE_MIN) and np.all(point <= OBSTACLE_MAX)

def segment_intersects_obstacle(p1, p2):
    # Axis-aligned rectangle collision (Cohen–Sutherland or Liang–Barsky)
    # For axis-aligned, check if segment crosses any of the four sides
    # Or, sample points along the segment and check if any are inside
    num_checks = 20
    for t in np.linspace(0, 1, num_checks):
        pt = p1 * (1 - t) + p2 * t
        if is_in_obstacle(pt):
            return True
    return False

def segment_clearance(p1, p2):
    # Minimal distance from segment to rectangle
    # Sample points along segment, compute min distance to rectangle
    num_checks = 50
    min_dist = float('inf')
    for t in np.linspace(0, 1, num_checks):
        pt = p1 * (1 - t) + p2 * t
        # Distance to rectangle
        dx = max(OBSTACLE_MIN[0] - pt[0], 0, pt[0] - OBSTACLE_MAX[0])
        dy = max(OBSTACLE_MIN[1] - pt[1], 0, pt[1] - OBSTACLE_MAX[1])
        dist = np.hypot(dx, dy)
        min_dist = min(min_dist, dist)
    return min_dist

def euclidean(p1, p2):
    return np.linalg.norm(p1 - p2)

def build_roadmap():
    roadmap = Roadmap()
    # Grid sampling
    grid_n = int(np.ceil(np.sqrt(NUM_SAMPLES)))
    xs = np.linspace(0, SQUARE_SIZE, grid_n)
    ys = np.linspace(0, SQUARE_SIZE, grid_n)
    points = np.array([(x, y) for x in xs for y in ys])
    # Reject points inside obstacle
    free_points = [p for p in points if not is_in_obstacle(p)]
    # Add start/goal
    free_points.append(START)
    free_points.append(GOAL)
    # Add nodes
    for p in free_points:
        roadmap.add_node(p)
    # KDTree for fast neighbor search
    node_positions = np.array([node.pos for node in roadmap.nodes])
    kdtree = KDTree(node_positions)
    # Connect neighbors
    for i, node in enumerate(roadmap.nodes):
        dists, idxs = kdtree.query(node.pos, k=K_NEIGHBORS + 1)  # +1 for self
        for j in idxs[1:]:  # skip self
            neighbor = roadmap.nodes[j]
            if segment_intersects_obstacle(node.pos, neighbor.pos):
                continue
            length = euclidean(node.pos, neighbor.pos)
            clearance = segment_clearance(node.pos, neighbor.pos)
            roadmap.add_edge(i, j, length, clearance)
    return roadmap

def dijkstra(roadmap, start_idx, goal_idx):
    import heapq
    num_nodes = len(roadmap.nodes)
    dist = [float('inf')] * num_nodes
    prev = [None] * num_nodes
    dist[start_idx] = 0.0
    queue = [(0.0, start_idx)]
    visited = set()
    while queue:
        cur_dist, u = heapq.heappop(queue)
        if u in visited:
            continue
        visited.add(u)
        if u == goal_idx:
            break
        for v, length, clearance in roadmap.nodes[u].neighbors:
            if v in visited:
                continue
            # Enforce minimum clearance
            if clearance < CLEARANCE_THRESHOLD:
                continue
            alt = cur_dist + length
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(queue, (alt, v))
    # Reconstruct path
    path = []
    u = goal_idx
    while u is not None:
        path.append(u)
        u = prev[u]
    path.reverse()
    return path

def rdp(points, epsilon):
    """Ramer–Douglas–Peucker algorithm for path simplification."""
    if len(points) < 3:
        return [points[0], points[-1]] if len(points) > 1 else list(points)
    start = points[0]
    end = points[-1]
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return [start, end]
    distances = []
    for i in range(1, len(points) - 1):
        pt = points[i]
        proj = start + np.dot(pt - start, line_vec) / (line_len ** 2) * line_vec
        dist = np.linalg.norm(pt - proj)
        distances.append(dist)
    max_dist = max(distances)
    idx = distances.index(max_dist) + 1
    if max_dist > epsilon:
        left = rdp(points[:idx+1], epsilon)
        right = rdp(points[idx:], epsilon)
        return left[:-1] + right  # list concatenation
    else:
        return [start, end]

def plot_roadmap_with_both_paths(roadmap, path, waypoints_simplified):
    fig, ax = plt.subplots(figsize=(8,8))
    rect = plt.Rectangle(OBSTACLE_MIN, OBSTACLE_SIZE, OBSTACLE_SIZE, color='red', alpha=0.3)
    ax.add_patch(rect)
    positions = np.array([node.pos for node in roadmap.nodes])
    ax.scatter(positions[:,0], positions[:,1], s=10, color='blue', label='Nodes')
    for idx1, idx2, length, clearance in roadmap.edges:
        p1 = roadmap.nodes[idx1].pos
        p2 = roadmap.nodes[idx2].pos
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='gray', linewidth=0.5)
    ax.scatter([START[0]], [START[1]], s=80, color='green', marker='*', label='Start')
    ax.scatter([GOAL[0]], [GOAL[1]], s=80, color='orange', marker='*', label='Goal')
    # Plot initial trajectory
    if path and len(path) > 1:
        path_points = np.array([roadmap.nodes[idx].pos for idx in path])
        ax.plot(path_points[:,0], path_points[:,1], color='magenta', linewidth=2, label='Initial Trajectory')
        ax.scatter(path_points[:,0], path_points[:,1], s=30, color='magenta')
    # Plot simplified trajectory
    if waypoints_simplified and len(waypoints_simplified) > 1:
        wps = np.array(waypoints_simplified)
        ax.plot(wps[:,0], wps[:,1], color='lime', linewidth=3, label='Simplified Trajectory')
        ax.scatter(wps[:,0], wps[:,1], s=60, color='lime')
    ax.set_xlim(0, SQUARE_SIZE)
    ax.set_ylim(0, SQUARE_SIZE)
    ax.set_aspect('equal')
    ax.legend(loc='lower right')  # Set legend to bottom right
    ax.set_title('Minimum distance roadmap with Initial and Simplified Trajectories')
    output_dir = os.path.join(os.path.dirname(__file__), '../output')
    os.makedirs(output_dir, exist_ok=True)
    image_path = os.path.join(output_dir, 'roadmap_with_both_paths_min_distance.png')
    plt.savefig(image_path)
    print(f"oadmap with both paths image saved to {image_path}")
    plt.show()

if __name__ == '__main__':
    roadmap = build_roadmap()
    # Start and goal are last two nodes added
    start_idx = len(roadmap.nodes) - 2
    goal_idx = len(roadmap.nodes) - 1
    path = dijkstra(roadmap, start_idx, goal_idx)
    waypoints = [roadmap.nodes[idx].pos for idx in path]
    print(f"Shortest path waypoints: {waypoints}")
    # Simplify waypoints using RDP
    epsilon = 0.3  # meters
    waypoints_simplified = rdp(np.array(waypoints), epsilon)
    print(f"Simplified waypoints: {waypoints_simplified}")
    # Save simplified waypoints to CSV
    output_dir = os.path.join(os.path.dirname(__file__), '../output')
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, 'trajectory_waypoints_min_distance.csv')
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y'])
        for wp in waypoints_simplified:
            writer.writerow([wp[0], wp[1]])
    print(f"Simplified waypoints saved to {csv_path}")
    # Only plot roadmap with both paths and legend in bottom right
    plot_roadmap_with_both_paths(roadmap, path, waypoints_simplified)
