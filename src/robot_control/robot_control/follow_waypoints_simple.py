import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import serial
import json
import time
import threading
import numpy as np
import math
import os

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) in degrees to quaternion
    
    Args:
        roll (float): Roll angle in degrees (rotation around X-axis)
        pitch (float): Pitch angle in degrees (rotation around Y-axis)  
        yaw (float): Yaw angle in degrees (rotation around Z-axis)
    
    Returns:
        tuple: (w, x, y, z) quaternion components
    """
    # Convert degrees to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)
    
    # Calculate half angles
    cy = math.cos(yaw_rad * 0.5)    # cos(yaw/2)
    sy = math.sin(yaw_rad * 0.5)    # sin(yaw/2)
    cp = math.cos(pitch_rad * 0.5)  # cos(pitch/2)
    sp = math.sin(pitch_rad * 0.5)  # sin(pitch/2)
    cr = math.cos(roll_rad * 0.5)   # cos(roll/2)
    sr = math.sin(roll_rad * 0.5)   # sin(roll/2)
    
    # Quaternion multiplication (ZYX rotation order)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return w, x, y, z

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # create publisher for IMU data
        self.imu_publisher = self.create_publisher(Imu, 'imu_data', 10)
        
        # initialize serial connection
        self.robot = None
        self.init_serial()
        
        # motor command state
        self.motor_command = {"T": 1, "L": 0.0, "R": 0.0}
        self.imu_request_command = {"T": 126}
        self.running = True
        
        # start threads
        self.motor_tx_thread = threading.Thread(target=self.send_motor_commands, daemon=True)
        self.motor_tx_thread.start()
        
        self.imu_tx_thread = threading.Thread(target=self.send_imu_requests, daemon=True)
        self.imu_tx_thread.start()
        
        self.rx_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.rx_thread.start()
        
        # Start waypoint navigation thread
        self.waypoint_thread = threading.Thread(target=self.follow_waypoints, daemon=True)
        self.waypoint_thread.start()
        
        self.get_logger().info('Motor Controller Node with IMU started')
        self.get_logger().info('Publishing IMU data on topic: imu_data')
        self.get_logger().info('Following waypoints from waypoints.txt')
        
    def init_serial(self):
        """initialize serial connection"""
        try:
            self.robot = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info('Serial connection established on /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().warn(f'Failed to establish serial connection: {str(e)}')
            self.robot = None
    
    def load_waypoints(self, filename):
        waypoints = []
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split(',')
                        if len(parts) == 3:
                            x, y, theta = map(float, parts)
                            waypoints.append((x, y, theta))
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
        return waypoints

    def follow_waypoints(self):
        # Load waypoints from file in the same directory
        waypoints_file = os.path.join('/home/ubuntu/ros2_ws/rubikpi_ros2/', 'waypoints.txt')
        waypoints = self.load_waypoints(waypoints_file)
        if not waypoints:
            self.get_logger().error('No waypoints loaded!')
            return
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints')
        # Assume starting at first waypoint
        current_x, current_y, current_theta = waypoints[0]
        speed = 0.12  # m/s (slower straight speed)
        turn_speed = 0.15  # rad/s (slower turn speed)
        for i in range(1, len(waypoints)):
            next_x, next_y, next_theta = waypoints[i]
            # Compute distance and angle to next waypoint
            dx = next_x - current_x
            dy = next_y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            # Move straight
            self.motor_command = {"T": 1, "L": speed, "R": speed}
            move_time = distance / speed if speed > 0 else 0
            self.get_logger().info(f'Moving {distance:.2f}m to WP{i} for {move_time:.2f}s')
            time.sleep(move_time)
            self.motor_command = {"T": 1, "L": 0.0, "R": 0.0}
            time.sleep(1)  # Pause at waypoint
            # Compute required turn
            target_angle = next_theta
            angle_diff = target_angle - current_theta
            # Normalize angle to [-pi, pi]
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            # Turn in place
            if abs(angle_diff) > 0.01:
                turn_direction = 1 if angle_diff > 0 else -1
                self.motor_command = {"T": 1, "L": -turn_direction * turn_speed, "R": turn_direction * turn_speed}
                turn_time = abs(angle_diff) / turn_speed
                self.get_logger().info(f'Turning {math.degrees(angle_diff):.1f} deg for {turn_time:.2f}s')
                time.sleep(turn_time)
                self.motor_command = {"T": 1, "L": 0.0, "R": 0.0}
                time.sleep(1)  # Pause after turn
            # Update current pose
            current_x, current_y, current_theta = next_x, next_y, next_theta
        self.get_logger().info('Finished all waypoints!')
        self.motor_command = {"T": 1, "L": 0.0, "R": 0.0}
    
    def send_motor_commands(self):
        """send motor commands to robot via serial at ~20Hz"""
        while self.running and rclpy.ok():
            if self.robot:
                try:
                    current_command = self.motor_command.copy()
                    command_str = json.dumps(current_command) + '\n'
                    self.robot.write(command_str.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f'Motor command serial write error: {str(e)}')
                    self.init_serial()
            time.sleep(0.05)  # 20Hz
    
    def send_imu_requests(self):
        """send IMU data requests to robot via serial at 50Hz"""
        while self.running and rclpy.ok():
            if self.robot:
                try:
                    command_str = json.dumps(self.imu_request_command) + '\n'
                    self.robot.write(command_str.encode('utf-8'))
                    
                except Exception as e:
                    self.get_logger().error(f'IMU request serial write error: {str(e)}')
                    # Don't reinit serial here to avoid conflict with motor thread
            time.sleep(0.02)  # 50Hz
    
    def receive_data(self):
        """receive and process data from robot"""
        buffer = ""
        while self.running and rclpy.ok():
            if self.robot:
                try:
                    # read available data
                    data = self.robot.read(1024)
                    if data:
                        buffer += data.decode('utf-8', errors='ignore')
                        
                        # process complete lines
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            
                            if line:
                                self.process_received_data(line)
                                
                except Exception as e:
                    self.get_logger().error(f'Serial read error: {str(e)}')
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
    
    def process_received_data(self, data_str):
        """process received data from robot"""
        try:
            data = json.loads(data_str)
            
            # check if this is IMU data (T: 1002)
            if isinstance(data, dict) and data.get('T') == 1002:
                self.process_imu_data(data)
                
        except json.JSONDecodeError:
            self.get_logger().debug(f'Received non-JSON data: {data_str}')
        except Exception as e:
            self.get_logger().error(f'Error processing received data: {str(e)}')
    
    def process_imu_data(self, imu_data):
        """process and publish IMU data"""
        try:
            # extract roll, pitch, yaw
            # print(imu_data)
            roll = float(imu_data.get('r', 0))    # degrees
            pitch = float(imu_data.get('p', 0))   # degrees  
            yaw = float(imu_data.get('y', 0))     # degrees
            
            # convert to quaternion
            qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
    
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # set orientation quaternion
            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            
            # angular velocity (rad/s) - convert from degrees/s if needed
            imu_msg.angular_velocity.x = float(imu_data.get('gx', 0))  # roll rate
            imu_msg.angular_velocity.y = float(imu_data.get('gy', 0))  # pitch rate  
            imu_msg.angular_velocity.z = float(imu_data.get('gz', 0))  # yaw rate
            
            # linear acceleration (m/s^2) - convert from mg to m/s^2 if needed
            imu_msg.linear_acceleration.x = float(imu_data.get('ax', 0)) * 9.81 / 1000.0
            imu_msg.linear_acceleration.y = float(imu_data.get('ay', 0)) * 9.81 / 1000.0
            imu_msg.linear_acceleration.z = float(imu_data.get('az', 0)) * 9.81 / 1000.0
            
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')
    
    def destroy_node(self):
        """Cleanup operations when node is destroyed"""
        self.running = False
        
        # Send stop command before closing (only if ROS context is still valid)
        try:
            if self.robot and rclpy.ok():
                stop_command = json.dumps({"T": 1, "L": 0.0, "R": 0.0}) + '\n'
                self.robot.write(stop_command.encode('utf-8'))
                self.get_logger().info('Robot stopped')
        except Exception as e:
            # Ignore errors during shutdown as context may already be invalid
            pass
        
        # Close serial connection
        try:
            if self.robot:
                self.robot.close()
                self.get_logger().info('Serial connection closed')
        except Exception as e:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        return 0

if __name__ == '__main__':
    main()