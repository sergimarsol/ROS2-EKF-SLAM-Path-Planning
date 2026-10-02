#!/usr/bin/env python3
import numpy as np
import time
import threading
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

"""
EKF-SLAM implementation for landmark-based localization
"""
class EKFSLAM:
    def __init__(self, motion_noise_scale=1.0):
        self.state = np.array([0.0, 0.0, 0.0])
        self.P = np.diag([0.1, 0.1, 0.05])
        self.landmark_dict = {}
        # Calibration constant for motion noise scaling
        self.motion_noise_scale = motion_noise_scale
        # Tunable motion noise parameters (lowered for more trust in motion)
        self.alpha = [0.05, 0.005, 0.005, 0.05]
        self.sigma_r = 0.1
        self.sigma_phi = 0.05

    def wrap_angle(self, angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle

    def get_motion_noise(self, v, omega, dt):
        sigma_v_sq = (self.alpha[0] * v**2 + self.alpha[1] * omega**2) * dt**2
        sigma_omega_sq = (self.alpha[2] * v**2 + self.alpha[3] * omega**2) * dt**2
        sigma_v_sq = max(sigma_v_sq, 0.001)
        sigma_omega_sq = max(sigma_omega_sq, 0.001)
        # Apply calibration constant
        Q_control = np.diag([sigma_v_sq, sigma_omega_sq]) * self.motion_noise_scale
        return Q_control

    def get_measurement_noise(self, robot_moving=False, turning=False):
        if turning:
            sigma_r = self.sigma_r * 1.5
            sigma_phi = self.sigma_phi * 3.0
        elif robot_moving:
            sigma_r = self.sigma_r * 1.2
            sigma_phi = self.sigma_phi * 1.5
        else:
            sigma_r = self.sigma_r
            sigma_phi = self.sigma_phi
        return np.diag([sigma_r**2, sigma_phi**2])

    def predict(self, v, omega, dt):
        x, y, theta = self.state[0:3]
        if abs(omega) < 1e-6:
            x_new = x + v * dt * np.cos(theta)
            y_new = y + v * dt * np.sin(theta)
            theta_new = theta
        else:
            radius = v / omega
            x_new = x - radius * np.sin(theta) + radius * np.sin(theta + omega * dt)
            y_new = y + radius * np.cos(theta) - radius * np.cos(theta + omega * dt)
            theta_new = theta + omega * dt
        theta_new = self.wrap_angle(theta_new)
        self.state[0:3] = [x_new, y_new, theta_new]
        if abs(omega) < 1e-6:
            F_x = np.array([
                [1, 0, -v*dt*np.sin(theta)],
                [0, 1,  v*dt*np.cos(theta)],
                [0, 0,  1]
            ])
        else:
            F_x = np.array([
                [1, 0, -v/omega * np.cos(theta) + v/omega * np.cos(theta + omega*dt)],
                [0, 1, -v/omega * np.sin(theta) + v/omega * np.sin(theta + omega*dt)],
                [0, 0, 1]
            ])
        if abs(omega) < 1e-6:
            G = np.array([
                [dt*np.cos(theta), 0],
                [dt*np.sin(theta), 0],
                [0, dt]
            ])
        else:
            G = np.array([
                [1/omega * (np.sin(theta + omega*dt) - np.sin(theta)), 
                 v/(omega**2) * (np.sin(theta) - np.sin(theta + omega*dt)) + v*dt/omega * np.cos(theta + omega*dt)],
                [1/omega * (-np.cos(theta + omega*dt) + np.cos(theta)), 
                 v/(omega**2) * (-np.cos(theta) + np.cos(theta + omega*dt)) + v*dt/omega * np.sin(theta + omega*dt)],
                [0, dt]
            ])
        n = len(self.state)
        F = np.eye(n)
        F[0:3, 0:3] = F_x
        Q_control = self.get_motion_noise(v, omega, dt)
        Q_state = G @ Q_control @ G.T
        Q_full = np.zeros((n, n))
        Q_full[0:3, 0:3] = Q_state
        self.P = F @ self.P @ F.T + Q_full

    def update(self, tag_id, range_meas, bearing_meas, robot_moving=False, robot_turning=False):
        if tag_id not in self.landmark_dict:
            self._initialize_landmark(tag_id, range_meas, bearing_meas)
            return
        lm_idx = self.landmark_dict[tag_id]
        x_r, y_r, theta_r = self.state[0:3]
        x_l, y_l = self.state[lm_idx:lm_idx+2]
        delta_x = x_l - x_r
        delta_y = y_l - y_r
        q = delta_x**2 + delta_y**2
        if q < 1e-6:
            return
        sqrt_q = np.sqrt(q)
        range_pred = sqrt_q
        bearing_pred = self.wrap_angle(np.arctan2(delta_y, delta_x) - theta_r)
        innovation = np.array([
            range_meas - range_pred,
            self.wrap_angle(bearing_meas - bearing_pred)
        ])
        n = len(self.state)
        H = np.zeros((2, n))
        H[0, 0] = -delta_x / sqrt_q
        H[0, 1] = -delta_y / sqrt_q
        H[0, 2] = 0
        H[1, 0] = delta_y / q
        H[1, 1] = -delta_x / q
        H[1, 2] = -1
        H[0, lm_idx]     = delta_x / sqrt_q
        H[0, lm_idx + 1] = delta_y / sqrt_q
        H[1, lm_idx]     = -delta_y / q
        H[1, lm_idx + 1] = delta_x / q
        R = self.get_measurement_noise(robot_moving, robot_turning)
        S = H @ self.P @ H.T + R
        try:
            np.linalg.cholesky(S)
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S += np.eye(2) * 1e-6
            try:
                K = self.P @ H.T @ np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return
        self.state = self.state + K @ innovation
        self.state[2] = self.wrap_angle(self.state[2])
        I = np.eye(len(self.state))
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        eigenvals = np.linalg.eigvals(self.P)
        if np.any(eigenvals < 0):
            self.P += np.eye(len(self.state)) * 1e-8

    def _initialize_landmark(self, tag_id, range_meas, bearing_meas):
        x_r, y_r, theta_r = self.state[0:3]
        P_robot = self.P[0:3, 0:3]
        x_l = x_r + range_meas * np.cos(theta_r + bearing_meas)
        y_l = y_r + range_meas * np.sin(theta_r + bearing_meas)
        G_robot = np.array([
            [1, 0, -range_meas * np.sin(theta_r + bearing_meas)],
            [0, 1,  range_meas * np.cos(theta_r + bearing_meas)]
        ])
        G_meas = np.array([
            [np.cos(theta_r + bearing_meas), -range_meas * np.sin(theta_r + bearing_meas)],
            [np.sin(theta_r + bearing_meas),  range_meas * np.cos(theta_r + bearing_meas)]
        ])
        R_init = self.get_measurement_noise(robot_moving=False, turning=False)
        P_landmark = G_robot @ P_robot @ G_robot.T + G_meas @ R_init @ G_meas.T
        self.state = np.append(self.state, [x_l, y_l])
        self.landmark_dict[tag_id] = len(self.state) - 2
        n_old = self.P.shape[0]
        P_new = np.zeros((n_old + 2, n_old + 2))
        P_new[0:n_old, 0:n_old] = self.P
        P_new[n_old:n_old+2, n_old:n_old+2] = P_landmark
        P_cross = G_robot @ P_robot
        P_new[n_old:n_old+2, 0:3] = P_cross
        P_new[0:3, n_old:n_old+2] = P_cross.T
        self.P = P_new

    def get_robot_pose(self):
        return self.state[0:3].copy()

    def get_landmarks(self):
        landmarks = {}
        for tag_id, idx in self.landmark_dict.items():
            if idx + 2 <= len(self.state):
                landmarks[tag_id] = self.state[idx:idx+2].copy()
        return landmarks

"""
Dead Reckoning implementation as fallback localization
"""
class DeadReckoning:
    def __init__(self, initial_pose=None):
        if initial_pose is None:
            self.pose = np.array([0.0, 0.0, 0.0])
        else:
            self.pose = np.array(initial_pose).copy()
        self.initialized = True

    def update(self, linear_vel, angular_vel, dt):
        self.pose[0] += linear_vel * np.cos(self.pose[2]) * dt
        self.pose[1] += linear_vel * np.sin(self.pose[2]) * dt
        self.pose[2] += angular_vel * dt
        self.pose[2] = self.wrap_angle(self.pose[2])

    def get_pose(self):
        return self.pose.copy()

    def set_pose(self, pose):
        self.pose = np.array(pose).copy()

    def wrap_angle(self, angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle

    def is_initialized(self):
        return self.initialized

"""
PID controller for differential drive robot
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
        self.I = np.array([0.0, 0.0])
        self.lastError = np.array([0.0, 0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState, drive_backwards):
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
        self.maximumValue = mv

    def update(self, currentState, drive_backwards):
        e = self.getError(currentState, self.target, drive_backwards)
        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep
        D = self.Kd * (e - self.lastError)
        result = P + self.I + D
        self.lastError = e
        if abs(result[0]) > self.maximumValue:
            result[0] = np.sign(result[0]) * self.maximumValue
        max_angular = 1.5
        if abs(result[1]) > max_angular:
            result[1] = np.sign(result[1]) * max_angular
        if abs(e[0]) < 0.05:
            result[0] = 0.0
        return result

def generate_movements(waypoints):
    """
    Given a list of waypoints [(x, y, theta), ...],
    returns a list of movement commands to move between them.
    Each movement is one of:
        - {'type': 'straight', 'distance': d}
        - {'type': 'turn', 'angle': a}
        - {'type': 'stop', 'duration': 2.0}
    """
    movements = []
    for i in range(len(waypoints) - 1):
        x1, y1, theta1 = waypoints[i]
        x2, y2, theta2 = waypoints[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        distance = np.hypot(dx, dy)
        desired_heading = np.arctan2(dy, dx)
        turn_angle = np.arctan2(np.sin(desired_heading - theta1), np.cos(desired_heading - theta1))
        if abs(turn_angle) > 1e-3:
            movements.append({'type': 'turn', 'angle': turn_angle})
        if distance > 1e-3:
            movements.append({'type': 'straight', 'distance': distance})
        final_turn = np.arctan2(np.sin(theta2 - desired_heading), np.cos(theta2 - desired_heading))
        if abs(final_turn) > 1e-3:
            movements.append({'type': 'turn', 'angle': final_turn})
        movements.append({'type': 'stop', 'duration': 2.0})
    return movements

class EkfSlamNode(Node):
    def __init__(self):
        super().__init__('ekf_slam_node_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Calibration constants for EKF motion and turn
        self.ekf = EKFSLAM(motion_noise_scale=0.30)
        self.turn_angle_scale = 1
        self.dead_reckoning = DeadReckoning()
        self.pid = PIDcontroller(1.2, 0.02, 0.01)
        self.latest_tag_transforms = {}
        self.ekf_trajectory = []
        self.dr_trajectory = []
        self.mid_trajectory = []
        # Waypoints and state variables
        self.waypoints = [
            np.array([0.0, 0.0, 0.0]),         # Start (bottom left)
            np.array([1.0, 0.0, np.pi/2]),     # Bottom right
            np.array([1.0, 1.0, np.pi]),       # Top right
            np.array([0.0, 1.0, -np.pi/2]),    # Top left
            np.array([0.0, 0.0, 0.0])          # Back to start
        ]
        # self.movements = generate_movements(self.waypoints)
        # Hardcoded movement sequence (as before)
        self.movements = [
            {'type': 'straight', 'distance': 1.0},
            {'type': 'turn', 'angle': np.pi/2},
            {'type': 'stop', 'duration': 2.0},
            {'type': 'straight', 'distance': 1.0},
            {'type': 'turn', 'angle': np.pi/2},
            {'type': 'stop', 'duration': 2.0},
            {'type': 'straight', 'distance': 1.0},
            {'type': 'turn', 'angle': np.pi/2},
            {'type': 'stop', 'duration': 2.0},
            {'type': 'straight', 'distance': 1.0},
            {'type': 'turn', 'angle': np.pi/2},
            {'type': 'stop', 'duration': 2.0},
        ]
        self.get_logger().info('Hardcoded movement sequence:')
        for idx, move in enumerate(self.movements):
            self.get_logger().info(f"  {idx}: {move}")
        self.current_move_idx = 0
        self.move_start_pose = None
        self.turn_start_angle = None
        self.stop_start_time = None
        # Subscribe to /tf for AprilTag detections
        from tf2_msgs.msg import TFMessage
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        # Timers for control and localization loops
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.localization_timer = self.create_timer(0.2, self.localization_update)  # 5 Hz
        self.current_waypoint_idx = 0
        # TF2 buffer and listener for transform lookups
        import tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('EkfSlamNode initialized with timers, waypoints, and TF2.')

        # Serial/motor control setup
        self.robot = None
        self.init_serial()
        self.motor_command = {"T": 1, "L": 0.0, "R": 0.0}
        self.running = True
        self.command_lock = threading.Lock()
        self.motor_tx_thread = threading.Thread(target=self.send_motor_commands, daemon=True)
        self.motor_tx_thread.start()

    def init_serial(self):
        """initialize serial connection"""
        try:
            import serial
            self.robot = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info('Serial connection established on /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error(f'Failed to establish serial connection: {str(e)}')
            self.robot = None

    def send_motor_commands(self):
        """send motor commands to robot via serial at ~20Hz"""
        while self.running and rclpy.ok():
            if self.robot:
                try:
                    with self.command_lock:
                        current_command = self.motor_command.copy()
                    command_str = json.dumps(current_command) + '\n'
                    self.robot.write(command_str.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f'Motor command serial write error: {str(e)}')
                    self.init_serial()
            time.sleep(0.05)  # 20Hz

    def start_spin(self, direction):
        """Start spinning in place. direction: +1 for left, -1 for right"""
        with self.command_lock:
            self.motor_command = {"T": 1, "L": -0.35 * direction, "R": 0.35 * direction}

    def stop_spin(self):
        """Stop spinning in place"""
        with self.command_lock:
            self.motor_command = {"T": 1, "L": 0.0, "R": 0.0}

    def destroy_node(self):
        """cleanup operations when node is destroyed"""
        self.running = False
        # send stop command before closing
        try:
            if self.robot:
                stop_command = json.dumps({"T": 1, "L": 0.0, "R": 0.0}) + '\n'
                self.robot.write(stop_command.encode('utf-8'))
                self.get_logger().info('Robot stopped')
                time.sleep(0.1)
        except Exception:
            pass
        # close serial connection
        try:
            if self.robot:
                self.robot.close()
                self.get_logger().info('Serial connection closed')
        except Exception:
            pass
        super().destroy_node()

    def tf_callback(self, msg):
        """Process incoming TF messages and store tag transforms."""
        for transform in msg.transforms:
            child = transform.child_frame_id
            if child.startswith('tag_'):
                try:
                    tag_id = int(child.split('_')[1])
                except Exception:
                    continue
                # Store the latest transform for this tag
                self.latest_tag_transforms[tag_id] = transform
                self.get_logger().info(f"Detected AprilTag: tag_{tag_id}")

    def wrap_angle(self, angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle

    def transform_to_range_bearing(self, tag_id):
        """
        Look up the transform from base_link to tag_{id} and return (range, bearing)
        Returns None if transform is not available.
        """
        try:
            tag_frame = f'tag_{tag_id}'
            t = self.tf_buffer.lookup_transform('base_link', tag_frame, rclpy.time.Time())
            dx = t.transform.translation.x
            dy = t.transform.translation.y
            range_meas = np.sqrt(dx**2 + dy**2)
            bearing_meas = np.arctan2(dy, dx)
            self.get_logger().info(f"TF lookup for tag_{tag_id}: dx={dx:.2f}, dy={dy:.2f}, range={range_meas:.2f}, bearing={np.degrees(bearing_meas):.1f} deg")
            return range_meas, bearing_meas
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed for tag_{tag_id}: {e}")
            return None

    def control_loop(self):
        # Use movement sequence from self.movements
        if self.current_move_idx >= len(self.movements):
            # All movements done, stop and save CSVs
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            import csv, os
            # Save in specified project_dir
            project_dir = os.path.expanduser('~/ros2_ws/src/ekf_slam/output')
            os.makedirs(project_dir, exist_ok=True)
            ekf_path = os.path.join(project_dir, 'ekf_trajectory.csv')
            dr_path = os.path.join(project_dir, 'dead_reckoning_trajectory.csv')
            mid_path = os.path.join(project_dir, 'midpoint_trajectory.csv')
            # Write all three trajectories, each row: x, y, theta (deg)
            def write_traj(path, traj, label):
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['x', 'y', 'theta_deg'])
                    for pose in traj:
                        writer.writerow([pose[0], pose[1], np.degrees(pose[2])])
                self.get_logger().info(f"Saved {label} trajectory to {path} ({len(traj)} points)")
            write_traj(ekf_path, self.ekf_trajectory, 'EKF')
            write_traj(dr_path, self.dr_trajectory, 'Dead Reckoning')
            write_traj(mid_path, self.mid_trajectory, 'Midpoint')
            # Save landmark positions to CSV
            landmarks = self.ekf.get_landmarks()
            landmark_path = os.path.join(project_dir, 'landmarks.csv')
            with open(landmark_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['tag_id', 'x', 'y'])
                for tag_id, pos in landmarks.items():
                    writer.writerow([tag_id, pos[0], pos[1]])
            self.get_logger().info(f"Saved landmark positions to {landmark_path}")
            self.get_logger().info(f'Trajectories saved to CSV: {ekf_path}, {dr_path}, {mid_path}')
            return
        move = self.movements[self.current_move_idx]
        ekf_pose = self.ekf.get_robot_pose()
        dr_pose = self.dead_reckoning.get_pose()
        mid_pose = 0.5 * (ekf_pose + dr_pose)
        self.ekf_trajectory.append(ekf_pose.copy())
        self.dr_trajectory.append(dr_pose.copy())
        self.mid_trajectory.append(mid_pose.copy())
        twist = Twist()
        lin_speed = 0.35
        ang_speed = 2.5
        # Calibration constant for straight distance
        straight_distance_calibration = 5.0
        # Calibration constant for turn angle
        turn_angle_calibration = 1.2
        # Calibration constant for small turn bursts
        turn_burst_scale = 0.85  # Make each burst slightly smaller
        # --- Closed-loop turn implementation ---
        if move['type'] == 'turn':
            # Initialize turn state if starting a new turn
            if self.turn_start_angle is None:
                self.turn_start_angle = mid_pose[2]
                # Apply calibration constant to commanded turn angle
                calibrated_angle = move['angle'] * turn_angle_calibration
                self.turn_target_angle = self.wrap_angle(self.turn_start_angle + calibrated_angle)
                self.turn_direction = np.sign(calibrated_angle)
                self.turning = True
                self.turn_step_active = False
                self.get_logger().info(f"[TURN INIT] Starting turn: target_angle={np.degrees(self.turn_target_angle):.1f} deg, direction={self.turn_direction}, calibrated_angle={np.degrees(calibrated_angle):.1f} deg")
            # Always check if a turn burst is needed
            if not self.turn_step_active:
                self.start_spin(self.turn_direction)
                self.turn_step_start_time = time.time()
                self.turn_step_active = True
                self.get_logger().info(f"[TURN BURST] start_spin called, direction={self.turn_direction}")
            # Stop after short burst (e.g. 0.25s * turn_burst_scale)
            burst_duration = 0.25 * turn_burst_scale
            if self.turn_step_active and (time.time() - self.turn_step_start_time > burst_duration):
                self.stop_spin()
                self.turn_step_active = False
                self.get_logger().info(f"[TURN BURST] stop_spin called after burst (duration={burst_duration:.2f}s)")
            # Check EKF/DR heading after each burst
            current_angle = mid_pose[2]
            angle_error = self.wrap_angle(self.turn_target_angle - current_angle)
            self.get_logger().info(f"[TURN FEEDBACK] Current angle: {np.degrees(current_angle):.1f} deg, Error: {np.degrees(angle_error):.1f} deg")
            if abs(angle_error) < np.deg2rad(8):
                # Target reached
                self.stop_spin()
                self.current_move_idx += 1
                self.turn_start_angle = None
                self.turn_step_active = False
                self.turning = False
                self.get_logger().info(f"[TURN DONE] Target reached, stopping turn.")
            elif abs(angle_error) > np.deg2rad(170):
                # Overshot, turn back
                self.turn_direction *= -1
                self.turn_step_active = False
                self.get_logger().info(f"[TURN OVERSHOT] Overshot, reversing direction to {self.turn_direction}")
            # Use actual angular velocity for EKF/DR during turn burst
            actual_spin_omega = 1.0 * self.turn_direction if self.turn_step_active else 0.0
            self.ekf.predict(0.0, actual_spin_omega, 0.1)
            self.dead_reckoning.update(0.0, actual_spin_omega, 0.1)
        elif move['type'] == 'straight':
            # ...existing code for straight...
            if self.move_start_pose is None:
                self.move_start_pose = mid_pose.copy()
            # Apply calibration constant to commanded distance
            calibrated_distance = move['distance'] * straight_distance_calibration
            dist = np.linalg.norm(mid_pose[:2] - self.move_start_pose[:2])
            if dist < calibrated_distance:
                twist.linear.x = lin_speed
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.current_move_idx += 1
                self.move_start_pose = None
            self.ekf.predict(twist.linear.x, twist.angular.z, 0.1)
            self.dead_reckoning.update(twist.linear.x, twist.angular.z, 0.1)
        elif move['type'] == 'stop':
            # ...existing code for stop...
            if self.stop_start_time is None:
                self.stop_start_time = time.time()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if time.time() - self.stop_start_time >= move['duration']:
                self.current_move_idx += 1
                self.stop_start_time = None
            self.ekf.predict(twist.linear.x, twist.angular.z, 0.1)
            self.dead_reckoning.update(twist.linear.x, twist.angular.z, 0.1)
        self.cmd_vel_pub.publish(twist)
        # Fix format specifier typo in DR logging
        self.get_logger().info(f"[CTRL] Move: {self.current_move_idx}, Type: {move['type']}, MidPose: [{mid_pose[0]:.2f}, {mid_pose[1]:.2f}, {np.degrees(mid_pose[2]):.1f} deg] | EKF: [{ekf_pose[0]:.2f}, {ekf_pose[1]:.2f}, {np.degrees(ekf_pose[2]):.1f} deg] | DR: [{dr_pose[0]:.2f}, {dr_pose[1]:.2f}, {np.degrees(dr_pose[2]):.1f} deg] | Cmd: [vx={twist.linear.x:.2f}, wz={twist.angular.z:.2f}]")

    def localization_update(self):
        # For each detected AprilTag, update EKF with range/bearing from TF
        n_updates = 0
        for tag_id in list(self.latest_tag_transforms.keys()):
            result = self.transform_to_range_bearing(tag_id)
            if result is not None:
                range_meas, bearing_meas = result
                self.ekf.update(tag_id, range_meas, bearing_meas)
                n_updates += 1
        # Short log for localization update
        pose = self.ekf.get_robot_pose()
        n_landmarks = len(self.ekf.get_landmarks())
        self.get_logger().info(f"[LOC] Pose: [{pose[0]:.2f}, {pose[1]:.2f}, {np.degrees(pose[2]):.1f} deg], Tags: {n_updates}, Landmarks: {n_landmarks}")

def main(args=None):
    rclpy.init(args=args)
    node = EkfSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
