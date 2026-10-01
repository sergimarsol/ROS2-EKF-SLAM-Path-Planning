#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
import numpy as np
from math import sin, cos, sqrt, atan2, pi
import time

class SequenceController:
    """
    Time-based sequence controller for exact movement execution.
    
    Sequence:
    1. 1m straight (3.52s) - reach WP1, wait
    2. 180° left turn, r=1m (3.6s) - reach WP2, wait  
    3. 1m straight (3.52s) - reach WP3, wait
    4. 180° left turn, r=1m (3.6s) - reach WP4, end
    
    Uses AprilTags only for small corrections, not primary navigation.
    """
    def __init__(self):
        # ===== Calibrated parameters from follow_waypoints_simple.py =====
        
        # STRAIGHT movement calibration
        # From original: straight_speed = 0.25, rvl = 0.8, alfa = 0.88, straight_duration_1m = 4.0 * alfa = 3.52s
        self.straight_speed_left = 0.25     # Left wheel speed
        self.straight_speed_right = 0.23    # Right wheel speed (0.25 * 0.8)
        self.straight_duration = 3.82       # CORRECTED: Match original calibrated duration
        
        # CURVED TURN calibration (180° turn with 1m radius)
        # From original: v_f2 = 0.25, v_diff2 = 0.44, beta2 = 1.8, base_duration_opt2 = 2.0 * beta2 = 3.6s
        # L = v_f2 - v_diff2/2 = 0.25 - 0.44/2 = 0.03, R = v_f2 + v_diff2/2 = 0.25 + 0.44/2 = 0.47
        self.curve_left_L = 0.10           # CORRECTED: Match original calibrated values
        self.curve_left_R = 0.29           # CORRECTED: Match original calibrated values
        self.turn_duration = 12.6           # CORRECTED: Match original calibrated duration
        
        # ===== Robot physical parameters for odometry =====
        self.wheel_radius = 0.0335  # meters (3.35cm radius)
        self.wheelbase = 0.127      # meters (12.7cm between wheels)
        
        # ===== Odometry tracking =====
        self.total_distance_traveled = 0.0  # Total distance since start
        self.robot_x = 0.0                  # Robot X position in world coordinates
        self.robot_y = 0.0                  # Robot Y position in world coordinates
        self.robot_theta = 0.0              # Robot heading in world coordinates
        
        # Waypoints for the rectangular path (for reference only)
        self.waypoints = [
            [0.0, 0.0, 0.0],      # WP0 (Start): (0,0,0) - facing east
            [1.0, 0.0, 0.0],      # WP1: (1,0,0) - facing east
            [1.0, 2.0, pi],       # WP2: (1,2,π) - facing west after 180° turn
            [0.0, 0.0, 0.0]       # WP3 (End): (0,0,0) - facing east after final turn
        ]

        # Movement sequence definition
        self.sequence = [
            {'type': 'straight', 'duration': self.straight_duration, 'description': '1m straight east'},
            {'type': 'turn_left', 'duration': self.turn_duration, 'description': '180° left turn'},
            {'type': 'straight', 'duration': self.straight_duration, 'description': '1m straight west'},
            {'type': 'turn_left', 'duration': self.turn_duration, 'description': '180° left turn'},
        ]
        
        # Current execution state
        self.current_sequence_idx = 0
        self.sequence_start_time = None
        self.is_executing_sequence = False
        self.is_paused = False
        self.pause_start_time = None
        self.pause_duration = 3.0  # 3 second pause between segments
        
        # Small correction gains for AprilTag feedback (INCREASED INFLUENCE)
        self.correction_gain_linear = 0.3   # Increased: Small linear correction (was 0.1)
        self.correction_gain_angular = 0.5  # Increased: Small angular correction (was 0.2)
        self.max_correction_linear = 0.15   # Increased: Max 15cm/s correction (was 5cm/s)
        self.max_correction_angular = 0.3   # Increased: Max 0.3 rad/s correction (was 0.1)
    
    def start_sequence(self):
        """Start the movement sequence."""
        self.current_sequence_idx = 0
        self.is_executing_sequence = True
        self.is_paused = False
        self.sequence_start_time = time.time()
        # Reset odometry
        self.total_distance_traveled = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        return f"Starting: {self.sequence[0]['description']}"
    
    def update_odometry(self, left_speed, right_speed, dt):
        """
        Update odometry based on wheel speeds and time.
        
        Args:
            left_speed: Left wheel speed (rad/s equivalent)
            right_speed: Right wheel speed (rad/s equivalent)
            dt: Time step (seconds)
        """
        # Convert wheel speeds to distances traveled
        left_distance_delta = left_speed * dt * self.wheel_radius
        right_distance_delta = right_speed * dt * self.wheel_radius
        
        # Calculate robot motion using differential drive kinematics
        linear_distance_delta = (left_distance_delta + right_distance_delta) / 2.0
        angular_delta = (right_distance_delta - left_distance_delta) / self.wheelbase
        
        # Update robot pose in world coordinates
        self.robot_x += linear_distance_delta * cos(self.robot_theta)
        self.robot_y += linear_distance_delta * sin(self.robot_theta)
        self.robot_theta += angular_delta
        self.robot_theta = (self.robot_theta + pi) % (2 * pi) - pi  # Normalize angle
        
        # Update total distance traveled
        self.total_distance_traveled += abs(linear_distance_delta)
    
    def get_odometry_summary(self):
        """Get current odometry state for logging."""
        return {
            'position': [self.robot_x, self.robot_y, self.robot_theta],
            'total_distance': self.total_distance_traveled,
            'position_str': f"({self.robot_x:.3f}, {self.robot_y:.3f}, {self.robot_theta*180/pi:.1f}°)",
            'distance_str': f"{self.total_distance_traveled:.3f}m"
        }
    
    def get_current_movement_commands(self, apriltag_correction=None, dt=0.1):
        """
        Get movement commands for current sequence step.
        
        Args:
            apriltag_correction: (dx, dy, dtheta) small corrections from AprilTag
            
        Returns:
            (left_speed, right_speed, status_msg) - direct motor speeds
        """
        if not self.is_executing_sequence:
            return 0.0, 0.0, "Sequence not started"
        
        if self.current_sequence_idx >= len(self.sequence):
            return 0.0, 0.0, "Sequence complete"
        
        # Handle pause between segments
        if self.is_paused:
            if time.time() - self.pause_start_time >= self.pause_duration:
                # End pause, start next movement
                self.is_paused = False
                self.current_sequence_idx += 1
                
                if self.current_sequence_idx >= len(self.sequence):
                    self.is_executing_sequence = False
                    return 0.0, 0.0, "Sequence complete"
                
                self.sequence_start_time = time.time()
                return 0.0, 0.0, f"Starting: {self.sequence[self.current_sequence_idx]['description']}"
            else:
                remaining = self.pause_duration - (time.time() - self.pause_start_time)
                return 0.0, 0.0, f"Pausing... {remaining:.1f}s remaining"
        
        # Execute current movement
        current_step = self.sequence[self.current_sequence_idx]
        elapsed_time = time.time() - self.sequence_start_time
        
        # Check if current movement is complete
        if elapsed_time >= current_step['duration']:
            # Start pause
            self.is_paused = True
            self.pause_start_time = time.time()
            return 0.0, 0.0, f"Completed: {current_step['description']}. Pausing..."
        
        # Generate base movement commands - DIRECT MOTOR SPEEDS
        if current_step['type'] == 'straight':
            base_left = self.straight_speed_left
            base_right = self.straight_speed_right
        elif current_step['type'] == 'turn_left':
            base_left = self.curve_left_L
            base_right = self.curve_left_R
        else:
            base_left = 0.0
            base_right = 0.0
        
        # Apply small AprilTag corrections if available
        if apriltag_correction is not None:
            dx, dy, dtheta = apriltag_correction
            
            # Convert corrections to differential drive adjustments
            if current_step['type'] == 'straight':
                # For straight movement, correct lateral drift and heading
                # Small speed adjustments to correct for drift
                speed_correction = np.clip(dx * self.correction_gain_linear * 0.5, -0.05, 0.05)
                angular_correction = np.clip(dtheta * self.correction_gain_angular * 0.2, -0.05, 0.05)
                
                final_left = base_left + speed_correction - angular_correction
                final_right = base_right + speed_correction + angular_correction
            else:
                # For turns, apply heading corrections
                angular_correction = np.clip(dtheta * self.correction_gain_angular * 0.1, -0.05, 0.05)
                final_left = base_left - angular_correction
                final_right = base_right + angular_correction
        else:
            final_left = base_left
            final_right = base_right
        
        # Update odometry with current commands
        self.update_odometry(final_left, final_right, dt)
        
        # Ensure motor speeds are within valid range
        final_left = np.clip(final_left, -0.5, 0.5)
        final_right = np.clip(final_right, -0.5, 0.5)
        
        progress = elapsed_time / current_step['duration'] * 100
        return final_left, final_right, f"{current_step['description']} - {progress:.1f}% complete"

    def calculate_waypoint_error(self, current_position, waypoint_idx):
        """
        Calculate position and orientation error at a waypoint.
        
        Args:
            current_position: [x, y, theta] current robot position
            waypoint_idx: Index of the waypoint to compare against
            
        Returns:
            (position_error, orientation_error) in meters and radians
        """
        if waypoint_idx >= len(self.waypoints):
            return None, None
            
        target_waypoint = self.waypoints[waypoint_idx]
        target_x, target_y, target_theta = target_waypoint
        current_x, current_y, current_theta = current_position
        
        # Calculate position error (Euclidean distance)
        position_error = sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
        
        # Calculate orientation error (normalized to [-π, π])
        orientation_error = current_theta - target_theta
        orientation_error = (orientation_error + pi) % (2 * pi) - pi
        
        return position_error, orientation_error

    def get_current_waypoint_index(self):
        """Get the waypoint index that corresponds to current sequence step."""
        # Map sequence steps to waypoint indices
        # Start at WP0, after first straight at WP1, after first turn at WP2, after second straight and turn back at WP3
        if self.current_sequence_idx == 0:
            return 0  # Starting at WP0
        elif self.current_sequence_idx == 1:
            return 1  # After first straight, should be at WP1
        elif self.current_sequence_idx == 2:
            return 2  # After first turn, should be at WP2
        elif self.current_sequence_idx >= 3:
            return 3  # After final movements, should be back at WP3 (origin)
        else:
            return 0

class AprilTagMap:
    """Stores known AprilTag positions in world coordinates."""
    def __init__(self):
        # Updated tag positions for the rectangular path
        # Tags positioned around the path perimeter
        self.tag_positions = {
            1: [0.0, -0.1, 0.0, pi/2],     # Tag 1: at (0,-0.1) facing north
            4: [2.0, 0.0, 0.0, pi],        # Tag 4: at (2,0) facing west
            5: [1.0, 2.1, 0.0, -pi/2],     # Tag 5: at (1,2.1) facing south
            6: [-1.0, 2.0, 0.0, 0],        # Tag 6: at (-1,2) facing east
        }
    
    def get_tag_position(self, tag_id):
        """Get world position of a tag by ID."""
        return self.tag_positions.get(tag_id, None)

class SequenceNavigationNode(Node):
    """Navigation node with time-based sequence control and AprilTag corrections."""
    
    def __init__(self):
        super().__init__('sequence_navigation_node')
        
        # Publishers and subscribers
        self.motor_cmd_pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.apriltag_sub = self.create_subscription(
            AprilTagDetectionArray, 
            '/detections', 
            self.apriltag_callback, 
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        
        # Navigation state (only for tracking, not primary control)
        self.current_state = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        
        # Sequence controller
        self.sequence_controller = SequenceController()
        
        # AprilTag-based corrections (INCREASED influence)
        self.tag_map = AprilTagMap()
        self.last_tag_detection = None
        self.last_tag_time = time.time()
        self.tag_timeout = 4.0  # Increased: corrections persist for 4 seconds (was 2.0)
        
        # Camera parameters
        self.camera_focal_length = 622.338
        self.tag_real_size = 0.1524  # 6 inches
        
        # Control timer
        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Odometry logging timer (every 0.5 seconds)
        self.odometry_log_timer = 0.5
        self.last_odometry_log_time = time.time()
        
        # Waypoint error tracking
        self.last_sequence_idx = -1  # Track when sequence step changes
        self.waypoint_errors_logged = set()  # Track which waypoints have been logged
        
        self.get_logger().info('Sequence Navigation - Direct Motor Control (Bypassing velocity_mapping)')
        self.get_logger().info('Movement sequence:')
        for i, step in enumerate(self.sequence_controller.sequence):
            self.get_logger().info(f'  Step {i+1}: {step["description"]} ({step["duration"]:.1f}s)')
        self.get_logger().info(f'Using AprilTags for corrections: {list(self.tag_map.tag_positions.keys())}')
        self.get_logger().info('Publishing motor commands directly to motor_commands topic')
        
        # Start the sequence
        msg = self.sequence_controller.start_sequence()
        self.get_logger().info(msg)

    def apriltag_callback(self, msg):
        """Process AprilTag detections for small corrections only."""
        if not msg.detections:
            return
            
        # Process the most confident detection
        best_detection = max(msg.detections, key=lambda d: d.decision_margin)
        
        if best_detection.decision_margin < 40.0:  # Lowered threshold for more detections (was 50.0)
            return
            
        tag_id = best_detection.id
        tag_world_pos = self.tag_map.get_tag_position(tag_id)
        
        if tag_world_pos is None:
            self.get_logger().warn(f'Unknown tag ID: {tag_id}')
            return
        
        # Calculate position correction from tag
        robot_position_estimate = self.estimate_robot_position_from_tag(best_detection, tag_world_pos)
        
        if robot_position_estimate is not None:
            # Calculate correction as difference from current dead reckoning estimate
            dx = robot_position_estimate[0] - self.current_state[0]
            dy = robot_position_estimate[1] - self.current_state[1]
            dtheta = robot_position_estimate[2] - self.current_state[2]
            dtheta = (dtheta + pi) % (2 * pi) - pi  # Normalize angle
            
            # Limit correction magnitude to prevent jumps (INCREASED LIMITS)
            dx = np.clip(dx, -0.5, 0.5)  # Increased: Max 50cm correction (was 30cm)
            dy = np.clip(dy, -0.5, 0.5)
            dtheta = np.clip(dtheta, -0.8, 0.8)  # Increased: Max ~45° correction (was ~28°)
            
            # Apply correction to position estimate (weighted fusion) - INCREASED WEIGHT
            correction_weight = min(best_detection.decision_margin / 80.0, 0.6)  # Max 60% weight (was 30%)
            self.current_state[0] += correction_weight * dx
            self.current_state[1] += correction_weight * dy
            self.current_state[2] += correction_weight * dtheta
            self.current_state[2] = (self.current_state[2] + pi) % (2 * pi) - pi
            
            # Store correction for movement adjustment
            correction = (dx, dy, dtheta)
            self.last_tag_detection = correction
            self.last_tag_time = time.time()
            
            self.get_logger().info(
                f'Tag {tag_id} correction: dx={dx:.3f}, dy={dy:.3f}, dθ={dtheta*180/pi:.1f}°, '
                f'weight={correction_weight:.2f}, margin={best_detection.decision_margin:.1f}'
            )

    def estimate_robot_position_from_tag(self, detection, tag_world_pos):
        """Estimate robot position from AprilTag detection using pinhole camera model with camera transform."""
        try:
            # Camera-to-robot transform (adjust these values based on your robot's camera mounting)
            # Typical values for a front-facing camera on a small robot:
            camera_offset_x = 0.05  # Camera is 5cm forward from robot center
            camera_offset_y = 0.0   # Camera is centered laterally
            camera_yaw_offset = 0.0  # Camera faces same direction as robot (0 radians)
            
            # Get tag center in image coordinates
            tag_center_x = detection.centre.x
            tag_center_y = detection.centre.y
            
            # Estimate distance from tag size in pixels
            corners = detection.corners
            width_pixels = sqrt((corners[1].x - corners[0].x)**2 + (corners[1].y - corners[0].y)**2)
            height_pixels = sqrt((corners[3].x - corners[0].x)**2 + (corners[3].y - corners[0].y)**2)
            tag_size_pixels = (width_pixels + height_pixels) / 2.0
            
            if tag_size_pixels < 20:  # Too small/far
                return None
                
            # Estimate distance using pinhole camera model
            distance = (self.tag_real_size * self.camera_focal_length) / tag_size_pixels
            
            # Estimate angle to tag from camera center
            image_center_x = 640  # Half of 1280
            pixel_offset_x = tag_center_x - image_center_x
            angle_to_tag = atan2(pixel_offset_x, self.camera_focal_length)
            
            # Calculate CAMERA position relative to tag first
            tag_x, tag_y, tag_z, tag_yaw = tag_world_pos
            
            # Camera angle relative to world when seeing the tag
            camera_angle_to_tag = self.current_state[2] + camera_yaw_offset + angle_to_tag
            
            # Camera position in world coordinates
            camera_x = tag_x - distance * cos(camera_angle_to_tag)
            camera_y = tag_y - distance * sin(camera_angle_to_tag)
            
            # Transform from camera position to robot center position
            # Apply inverse transform: robot = camera - camera_offset_in_robot_frame
            robot_heading = self.current_state[2]  # Current robot heading
            
            # Camera offset in world coordinates
            offset_world_x = camera_offset_x * cos(robot_heading) - camera_offset_y * sin(robot_heading)
            offset_world_y = camera_offset_x * sin(robot_heading) + camera_offset_y * cos(robot_heading)
            
            # Robot center position
            robot_x = camera_x - offset_world_x
            robot_y = camera_y - offset_world_y
            robot_theta = self.current_state[2]  # Keep current heading estimate
            
            return [robot_x, robot_y, robot_theta]
            
        except Exception as e:
            self.get_logger().error(f'Error estimating position from tag: {e}')
            return None

    def update_dead_reckoning(self, linear_vel, angular_vel):
        """Update robot pose estimate using dead reckoning."""
        self.current_state[0] += linear_vel * cos(self.current_state[2]) * self.dt
        self.current_state[1] += linear_vel * sin(self.current_state[2]) * self.dt
        self.current_state[2] += angular_vel * self.dt
        self.current_state[2] = (self.current_state[2] + pi) % (2 * pi) - pi

    def broadcast_tf(self):
        """Broadcast TF transform."""
        current_time = self.get_clock().now()
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = float(self.current_state[0])
        t.transform.translation.y = float(self.current_state[1])
        t.transform.translation.z = 0.0
        
        yaw = self.current_state[2]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin(yaw / 2.0)
        t.transform.rotation.w = cos(yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

    def control_loop(self):
        """Main control loop with time-based sequence execution."""
        # Get AprilTag correction if available and recent
        apriltag_correction = None
        if self.last_tag_detection is not None:
            if time.time() - self.last_tag_time < self.tag_timeout:
                apriltag_correction = self.last_tag_detection
        
        # Get movement commands from sequence controller (pass dt for odometry)
        left_speed, right_speed, status_msg = self.sequence_controller.get_current_movement_commands(
            apriltag_correction, self.dt
        )
        
        # Check for waypoint completion and calculate errors
        current_sequence_idx = self.sequence_controller.current_sequence_idx
        
        # Detect when robot completes a movement and is pausing (reaches a waypoint)
        if (self.sequence_controller.is_paused and 
            current_sequence_idx != self.last_sequence_idx and
            current_sequence_idx not in self.waypoint_errors_logged):
            
            # Calculate waypoint error
            waypoint_idx = self.sequence_controller.get_current_waypoint_index()
            if waypoint_idx < len(self.sequence_controller.waypoints):
                position_error, orientation_error = self.sequence_controller.calculate_waypoint_error(
                    self.current_state, waypoint_idx
                )
                
                if position_error is not None and orientation_error is not None:
                    waypoint = self.sequence_controller.waypoints[waypoint_idx]
                    self.get_logger().info(
                        f'WAYPOINT {waypoint_idx} ERROR ANALYSIS:\n'
                        f'  Target: ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]*180/pi:.0f}°)\n'
                        f'  Actual: ({self.current_state[0]:.2f}, {self.current_state[1]:.2f}, {self.current_state[2]*180/pi:.0f}°)\n'
                        f'  Position Error: {position_error:.3f} m\n'
                        f'  Orientation Error: {orientation_error*180/pi:.1f}°'
                    )
                    
                    # Mark this waypoint as logged
                    self.waypoint_errors_logged.add(current_sequence_idx)
        
        # Update tracking variable
        self.last_sequence_idx = current_sequence_idx
        
        # Create and publish motor command message
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(left_speed), float(right_speed)]
        self.motor_cmd_pub.publish(motor_msg)
        
        # Calculate equivalent linear and angular velocities for dead reckoning
        # Using differential drive kinematics: v = (vL + vR)/2, ω = (vR - vL)/wheelbase
        wheelbase = 0.127  # meters
        linear_vel = (left_speed + right_speed) / 2.0
        angular_vel = (right_speed - left_speed) / wheelbase
        
        # Update position estimate using original dead reckoning
        self.update_dead_reckoning(linear_vel, angular_vel)
        
        # Broadcast TF
        self.broadcast_tf()
        
        # Get odometry summary for logging (from sequence controller's wheel odometry)
        odometry = self.sequence_controller.get_odometry_summary()
        
        # Log odometry every 0.5 seconds
        current_time = time.time()
        if current_time - self.last_odometry_log_time >= self.odometry_log_timer:
            self.get_logger().info(
                f'ODOMETRY: Position {odometry["position_str"]} | '
                f'Total Distance: {odometry["distance_str"]} | '
                f'Dead Reckoning: ({self.current_state[0]:.3f}, {self.current_state[1]:.3f}, {self.current_state[2]*180/pi:.1f}°) | '
                f'Status: {status_msg}'
            )
            self.last_odometry_log_time = current_time
        
        # Log status (less frequent - every 2 seconds)
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time > 2.0:
                self.get_logger().info(
                    f'Status: {status_msg} | '
                    f'Pos: ({self.current_state[0]:.2f}, {self.current_state[1]:.2f}, {self.current_state[2]*180/pi:.0f}°) | '
                    f'Motors: L={left_speed:.3f}, R={right_speed:.3f}'
                )
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()
        
        # Check if sequence is complete
        if not self.sequence_controller.is_executing_sequence and self.sequence_controller.current_sequence_idx >= len(self.sequence_controller.sequence):
            self.get_logger().info('Sequence complete! Stopping robot.')
            motor_msg = Float32MultiArray()
            motor_msg.data = [0.0, 0.0]  # Zero motor speeds
            self.motor_cmd_pub.publish(motor_msg)

    def stop_robot(self):
        """Stop the robot."""
        motor_msg = Float32MultiArray()
        motor_msg.data = [0.0, 0.0]
        self.motor_cmd_pub.publish(motor_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SequenceNavigationNode()
    
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
