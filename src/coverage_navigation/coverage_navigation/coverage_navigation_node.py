#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import numpy as np
import os 
import yaml
import time
import math
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import csv

"""
Virtual Obstacle Detector for Subsumption Architecture
Detects when robot is too close to arena boundaries (virtual walls)
"""
class VirtualObstacleDetector:
    def __init__(self, arena_bounds, safety_margin=0.3):
        """
        arena_bounds: dict with 'xmin', 'xmax', 'ymin', 'ymax' in meters
        safety_margin: distance from boundary to trigger obstacle detection
        """
        self.xmin = arena_bounds['xmin']
        self.xmax = arena_bounds['xmax']
        self.ymin = arena_bounds['ymin']
        self.ymax = arena_bounds['ymax']
        self.safety_margin = safety_margin
        
    def detect_obstacle(self, current_pose):
        """
        Check if robot is too close to any boundary
        Returns: (obstacle_detected, avoidance_direction)
            obstacle_detected: bool
            avoidance_direction: angular velocity direction to turn away from obstacle
        """
        x, y, theta = current_pose
        
        # Check each boundary
        dist_to_left = x - self.xmin
        dist_to_right = self.xmax - x
        dist_to_bottom = y - self.ymin
        dist_to_top = self.ymax - y
        
        min_dist = min(dist_to_left, dist_to_right, dist_to_bottom, dist_to_top)
        
        if min_dist < self.safety_margin:
            # Determine which boundary is closest and turn away
            if min_dist == dist_to_left:
                # Too close to left wall, turn right (negative angular)
                return True, -0.8
            elif min_dist == dist_to_right:
                # Too close to right wall, turn left (positive angular)
                return True, 0.8
            elif min_dist == dist_to_bottom:
                # Too close to bottom wall, turn based on current heading
                return True, 0.8 if theta > 0 else -0.8
            else:  # dist_to_top
                # Too close to top wall, turn based on current heading
                return True, -0.8 if theta > 0 else 0.8
        
        return False, 0.0

"""
The class of the pid controller for differential drive robot.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0, 0.0])
        self.lastError = np.array([0.0, 0.0])
        self.timestep = 0.1
        self.maximumValue = 0.2

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0, 0.0]) 
        self.lastError = np.array([0.0, 0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState, drive_backwards):
        """
        return the error between current and target state
        for differential drive: distance error and heading error
        """
        delta_x = targetState[0] - currentState[0]
        delta_y = targetState[1] - currentState[1]
        
        distance = np.sqrt(delta_x**2 + delta_y**2)
        
        angle_to_target = np.arctan2(delta_y, delta_x)
        
        if drive_backwards:
            desired_heading = angle_to_target + np.pi
            desired_heading = (desired_heading + np.pi) % (2 * np.pi) - np.pi
            distance = -distance
        else:
            desired_heading = angle_to_target
        
        heading_error = desired_heading - currentState[2]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        
        if abs(distance) < 0.05:
            heading_error = targetState[2] - currentState[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            distance = 0.0
        
        return np.array([distance, heading_error])

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState, drive_backwards):
        """
        calculate the update value based on PID control
        Returns: [linear_velocity, angular_velocity]
        """
        e = self.getError(currentState, self.target, drive_backwards)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        if abs(result[0]) > self.maximumValue:
            result[0] = np.sign(result[0]) * self.maximumValue
            
        max_angular = 1.5 # rad/s
        if abs(result[1]) > max_angular:
            result[1] = np.sign(result[1]) * max_angular
        
        if abs(e[0]) < 0.05:
            result[0] = 0.0
        return result

class Hw5SolutionNode(Node):
    def __init__(self):
        super().__init__('coverage_navigation_node_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        
        self.tag_positions = {}
        
        # Load waypoints from CSV file
        # Try to load lawnmower waypoints from the generated CSV
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Look next to the module first, then in the package-level output/ folder
        # (where lawnmower_trajectory.py writes it; found with `colcon build --symlink-install`)
        lawnmower_csv = os.path.join(script_dir, 'output', 'turn_waypoints.csv')
        package_csv = os.path.join(script_dir, '..', 'output', 'turn_waypoints.csv')
        if not os.path.exists(lawnmower_csv) and os.path.exists(package_csv):
            lawnmower_csv = os.path.abspath(package_csv)
        
        if os.path.exists(lawnmower_csv):
            self.waypoints = self.load_waypoints_from_csv(lawnmower_csv)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from lawnmower trajectory')
        else:
            # Fallback to hardcoded waypoints for testing
            self.get_logger().warn(f'Lawnmower CSV not found at {lawnmower_csv}, using default waypoints')
            self.waypoints = [
                [0.5,0.5,0.0],
                [2.0,2.1818181818181817,0.0],
                [1.8181818181818183,4.545454545454546,0.0],
                [3.4545454545454546,6.181818181818182,0.0],
                [5.818181818181818,6.0,0.0],
                [7.5,7.5,0.0]]

        self.pid = PIDcontroller(0.8, 0.01, 0.005)
        
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.obs_current_state = np.array([0.0, 0.0, 0.0])
        
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.tolerance = 0.15
        self.angle_tolerance = 0.1
        
        self.last_tag_detection_time = 0.0
        self.using_tag_localization = False
        self.tag_initialized = False
        
        self.load_tag_configurations()
        
        self.drive_backwards = False
        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.localization_timer = self.create_timer(0.1, self.localization_update) 
        
        self.stage = 'rotate_to_goal'
        self.stage_pid = PIDcontroller(0.8, 0.01, 0.005)
        self.fixed_rotation_vel = 0.785
        
        # ===== SUBSUMPTION ARCHITECTURE =====
        # Arena bounds for virtual obstacle detection (7ft x 7ft = ~2.13m x 2.13m)
        ft2m = 0.3048
        arena_size = 7.0 * ft2m  # 2.1336 meters
        arena_bounds = {
            'xmin': 0.0,
            'xmax': arena_size,
            'ymin': 0.0,
            'ymax': arena_size
        }
        
        # Initialize virtual obstacle detector with safety margin
        self.obstacle_detector = VirtualObstacleDetector(
            arena_bounds=arena_bounds,
            safety_margin=0.25  # 25cm from boundary triggers avoidance
        )
        
        # Subsumption behavior state
        self.avoiding_obstacle = False
        self.avoidance_start_time = 0.0
        self.avoidance_duration = 1.5  # seconds to avoid before resuming waypoint following
        self.avoidance_angular_vel = 0.0
        
        self.get_logger().info('=== HW5 SUBSUMPTION ARCHITECTURE INITIALIZED ===')
        self.get_logger().info(f'Arena bounds: {arena_size:.2f}m x {arena_size:.2f}m')
        self.get_logger().info(f'Obstacle detection margin: {self.obstacle_detector.safety_margin:.2f}m')
        self.get_logger().info('Behavior priorities: 1) Collision Avoidance, 2) Waypoint Following')
        
    def load_waypoints_from_csv(self, csv_path):
        waypoints = []
        try:
            with open(csv_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader)  # skip header
                for row in reader:
                    x = float(row[0])
                    y = float(row[1])
                    waypoints.append([x, y, 0.0])  # set theta=0.0 by default
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from CSV: {str(e)}')
        return np.array(waypoints)

    def load_tag_configurations(self):
        """Load AprilTag positions and orientations from YAML file"""
        try:
            package_share_dir = get_package_share_directory('coverage_navigation')
            yaml_path = os.path.join(package_share_dir, 'configs', 'apriltags_position.yaml')
            
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
            
            tags_data = data.get('apriltags', [])
            
            for tag in tags_data:
                tag_id = tag.get('id')
                if tag_id is None:
                    continue
                
                self.tag_positions[tag_id] = {
                    'x': float(tag['x']),
                    'y': float(tag['y']),
                    'z': float(tag['z']),
                    'qx': float(tag['qx']),
                    'qy': float(tag['qy']),
                    'qz': float(tag['qz']),
                    'qw': float(tag['qw'])
                }
                
                self.get_logger().info(
                    f'Loaded tag {tag_id}: pos=({tag["x"]:.2f}, {tag["y"]:.2f}, {tag["z"]:.2f})'
                )
                
        except Exception as e:
            self.get_logger().error(f'Failed to load tag configurations: {str(e)}')
        
    def update_dead_reckoning(self, linear_vel, angular_vel):
        """
        Update robot pose using dead reckoning
        """
        self.current_state[0] += linear_vel * np.cos(self.current_state[2]) * self.dt
        self.current_state[1] += linear_vel * np.sin(self.current_state[2]) * self.dt
        self.current_state[2] += angular_vel * self.dt
        self.current_state[2] = (self.current_state[2] + np.pi) % (2 * np.pi) - np.pi
        
    def broadcast_tf(self):
        """
        Broadcast TF transform from odom to base_link
        """
        current_time = self.get_clock().now()
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.current_state[0]
        t.transform.translation.y = self.current_state[1]
        t.transform.translation.z = 0.0
        
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.current_state[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def get_desired_heading_to_goal(self, current_wp, drive_backwards):
        """
        Get the desired heading to face towards (or away from) the goal
        """
        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        angle_to_target = np.arctan2(delta_y, delta_x)
        
        if drive_backwards:
            desired_heading = angle_to_target + np.pi
            desired_heading = (desired_heading + np.pi) % (2 * np.pi) - np.pi
        else:
            desired_heading = angle_to_target
        
        return desired_heading
    
    def get_rotation_direction(self, heading_error):
        """
        Determine rotation direction based on heading error.
        Returns: angular velocity with fixed magnitude but correct direction
        """
        if heading_error > 0:
            return self.fixed_rotation_vel
        else:
            return -self.fixed_rotation_vel
        
    def compute_and_publish_robot_pose_from_tag(self, tag_id, tag_observation):
        """Compute robot pose from tag observation and publish it"""
        tag_map = self.tag_positions[tag_id]
        
        tag_map_pos = np.array([tag_map['x'], tag_map['y'], tag_map['z']])
        tag_map_rot = Rotation.from_quat([tag_map['qx'], tag_map['qy'], tag_map['qz'], tag_map['qw']])
        
        obs_pos = np.array([
            tag_observation.transform.translation.x,
            tag_observation.transform.translation.y,
            tag_observation.transform.translation.z
        ])
        obs_rot = Rotation.from_quat([
            tag_observation.transform.rotation.x,
            tag_observation.transform.rotation.y,
            tag_observation.transform.rotation.z,
            tag_observation.transform.rotation.w
        ])
        
        tag_to_robot_rot = obs_rot.inv()
        tag_to_robot_pos = -tag_to_robot_rot.apply(obs_pos)
        
        robot_map_rot = tag_map_rot * tag_to_robot_rot
        robot_map_pos = tag_map_pos + tag_map_rot.apply(tag_to_robot_pos)
        
        yaw = robot_map_rot.as_euler('xyz')[2]
        self.current_state = np.array([robot_map_pos[0], robot_map_pos[1], yaw])
        self.obs_current_state = np.array([robot_map_pos[0], robot_map_pos[1], yaw])
        
        self.get_logger().info(
            f'Updated pose from tag {tag_id}: '
            f'pos=({robot_map_pos[0]:.3f}, {robot_map_pos[1]:.3f}), yaw={yaw:.3f}'
        )

    def store_trajectory_to_csv(self, csv_path, trajectory):
        """
        Store the trajectory as x, y, t in a CSV file.
        trajectory: list of (x, y, t)
        """
        try:
            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['x', 'y', 't'])
                for x, y, t in trajectory:
                    writer.writerow([x, y, t])
            self.get_logger().info(f'Trajectory saved to {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save trajectory: {str(e)}')

    def control_loop(self):
        """
        Main control loop with SUBSUMPTION ARCHITECTURE:
        Priority 1 (Highest): Collision Avoidance
        Priority 2 (Lower): Waypoint Following (rotate to goal, drive, rotate to orientation)
        """
        if not hasattr(self, 'trajectory_log'):
            self.trajectory_log = []
            self.trajectory_start_time = time.time()
        current_time = time.time() - self.trajectory_start_time
        self.trajectory_log.append((self.current_state[0], self.current_state[1], current_time))

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('All waypoints reached! Stopping robot.')
            self.stop_robot()
            self.broadcast_tf()
            # Save trajectory to CSV
            output_csv = os.path.expanduser('~/ros2_ws/src/coverage_navigation/output/actual_trajectory.csv')
            self.store_trajectory_to_csv(output_csv, self.trajectory_log)
            return

        twist_msg = Twist()
        
        # ===== SUBSUMPTION LAYER 1 (HIGHEST PRIORITY): COLLISION AVOIDANCE =====
        obstacle_detected, avoidance_direction = self.obstacle_detector.detect_obstacle(self.current_state)
        
        if obstacle_detected:
            if not self.avoiding_obstacle:
                # Just detected obstacle - start avoidance behavior
                self.avoiding_obstacle = True
                self.avoidance_start_time = time.time()
                self.avoidance_angular_vel = avoidance_direction
                self.get_logger().warn(
                    f'OBSTACLE DETECTED! Activating collision avoidance. '
                    f'Pos: ({self.current_state[0]:.2f}, {self.current_state[1]:.2f})'
                )
            
            # Execute avoidance behavior: stop forward motion and rotate away
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = float(self.avoidance_angular_vel)
            
            # Update dead reckoning and publish
            self.update_dead_reckoning(twist_msg.linear.x, twist_msg.angular.z)
            self.broadcast_tf()
            self.cmd_vel_pub.publish(twist_msg)
            return  # Skip waypoint following behavior
        
        # Check if we should exit avoidance mode
        if self.avoiding_obstacle:
            elapsed = time.time() - self.avoidance_start_time
            if elapsed < self.avoidance_duration:
                # Continue avoidance even if obstacle cleared (for minimum duration)
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = float(self.avoidance_angular_vel)
                self.update_dead_reckoning(twist_msg.linear.x, twist_msg.angular.z)
                self.broadcast_tf()
                self.cmd_vel_pub.publish(twist_msg)
                return
            else:
                # Avoidance complete, resume waypoint following
                self.avoiding_obstacle = False
                self.get_logger().info('Collision avoidance complete. Resuming waypoint following.')
        
        # ===== SUBSUMPTION LAYER 2 (LOWER PRIORITY): WAYPOINT FOLLOWING =====
        current_wp = self.waypoints[self.current_waypoint_idx]
        
        if not self.waypoint_reached:
            self.pid.setTarget(current_wp)
            self.drive_backwards = False
            self.waypoint_reached = True
            self.stage = 'rotate_to_goal'
            self.stage_pid.setTarget(current_wp)

        delta_x = current_wp[0] - self.obs_current_state[0]
        delta_y = current_wp[1] - self.obs_current_state[1]
        position_error = np.sqrt(delta_x**2 + delta_y**2)
        
        # Stage 1: Rotate to face the goal (or away if driving backwards)
        if self.stage == 'rotate_to_goal':
            desired_heading = self.get_desired_heading_to_goal(current_wp, self.drive_backwards)
            heading_error = desired_heading - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            
            if abs(heading_error) < 0.05:
                self.stage = 'drive'
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
        # Stage 2: Drive towards/away from the goal
        elif self.stage == 'drive':
            position_error = np.sqrt(delta_x**2 + delta_y**2)
            if position_error < self.tolerance:
                self.stage = 'rotate_to_orient'
                twist_msg.linear.x = 0.0
            else:
                desired_heading = self.get_desired_heading_to_goal(current_wp, self.drive_backwards)
                heading_error = desired_heading - self.current_state[2]
                heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
                if abs(heading_error) > 0.2:
                    self.stage = 'rotate_to_goal'
                    twist_msg.linear.x = 0.0
                else:
                    update_value = self.pid.update(self.current_state, self.drive_backwards)
                    twist_msg.linear.x = float(update_value[0])
        
        # Stage 3: Rotate to target orientation
        elif self.stage == 'rotate_to_orient':
            heading_error = current_wp[2] - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            if abs(heading_error) < self.angle_tolerance:
                self.current_waypoint_idx += 1
                self.waypoint_reached = False
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
        # Only update dead reckoning if not using tag localization
        # if not self.using_tag_localization:
        self.update_dead_reckoning(twist_msg.linear.x, twist_msg.angular.z)
        self.broadcast_tf()
        self.cmd_vel_pub.publish(twist_msg)
        
        
    def localization_update(self):
        """Main localization update - tries AprilTag first, then dead reckoning"""
        current_time = time.time()
        tag_detected = False
        closest_tag_id = None
        closest_observation = None
        closest_distance = float('inf')
        for tag_id in self.tag_positions.keys():
            try:
                tag_frame = f'tag_{tag_id}'
                observation = self.tf_buffer.lookup_transform('base_link', tag_frame, rclpy.time.Time())
                transform_time = rclpy.time.Time.from_msg(observation.header.stamp)
                time_diff = (self.get_clock().now() - transform_time).nanoseconds / 1e9
                if time_diff > 0.25:
                    continue
                dx = observation.transform.translation.x
                dy = observation.transform.translation.y
                dz = observation.transform.translation.z
                distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_tag_id = tag_id
                    closest_observation = observation
            except Exception:
                continue
        
        if closest_observation is not None:
            self.compute_and_publish_robot_pose_from_tag(closest_tag_id, closest_observation)
            tag_detected = True
            self.last_tag_detection_time = current_time
            self.using_tag_localization = True
        else:
            tag_detected = False
            time_since_last_tag = current_time - self.last_tag_detection_time
            if time_since_last_tag > 1.0:  # 1 second timeout
                self.using_tag_localization = False
    

    def stop_robot(self):
        """
        Stop the robot by publishing zero velocities
        """
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Hw5SolutionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by keyboard interrupt')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
