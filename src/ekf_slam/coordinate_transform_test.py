#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation

def test_coordinate_transformation():
    """Test the coordinate transformation mathematically"""
    
    print("=== COORDINATE TRANSFORMATION VERIFICATION ===\n")
    
    # Camera TF parameters from camera_tf.py
    camera_translation = np.array([0.0675, 0.0, 0.035])  # x, y, z in base_link frame
    camera_quaternion = np.array([-0.5, 0.5, -0.5, 0.5])  # x, y, z, w (scipy format)
    
    print(f"Camera position in base_link: {camera_translation}")
    print(f"Camera quaternion (x,y,z,w): {camera_quaternion}")
    
    # Convert quaternion to rotation matrix
    r = Rotation.from_quat(camera_quaternion)
    rotation_matrix = r.as_matrix()
    
    print(f"\nCamera rotation matrix:")
    print(rotation_matrix)
    
    # Analyze the rotation
    euler_angles = r.as_euler('xyz', degrees=True)
    print(f"Euler angles (degrees): roll={euler_angles[0]:.1f}°, pitch={euler_angles[1]:.1f}°, yaw={euler_angles[2]:.1f}°")
    
    print("\n=== COORDINATE FRAME ANALYSIS ===")
    
    # Test basis vectors to understand the transformation
    # Camera frame basis vectors in camera coordinates
    camera_x = np.array([1, 0, 0])  # camera x-axis (right)
    camera_y = np.array([0, 1, 0])  # camera y-axis (down) 
    camera_z = np.array([0, 0, 1])  # camera z-axis (forward/depth)
    
    # Transform to base_link coordinates
    base_x = rotation_matrix @ camera_x
    base_y = rotation_matrix @ camera_y  
    base_z = rotation_matrix @ camera_z
    
    print(f"Camera X-axis (right) in base_link: {base_x}")
    print(f"Camera Y-axis (down) in base_link: {base_y}")
    print(f"Camera Z-axis (forward) in base_link: {base_z}")
    
    print("\n=== APRIL TAG SCENARIO TEST ===")
    
    # Simulate an AprilTag in front of the robot
    # Tag is 1 meter ahead, 0.5m to the right of the robot
    tag_position_world = np.array([1.0, 0.5, 0.0])  # in base_link coordinates
    
    print(f"AprilTag actual position (base_link): {tag_position_world}")
    
    # What the camera would see this tag as:
    # 1. Translate to camera origin
    tag_relative_to_camera = tag_position_world - camera_translation
    
    # 2. Rotate to camera frame
    tag_in_camera_frame = rotation_matrix.T @ tag_relative_to_camera
    
    print(f"Tag relative to camera center: {tag_relative_to_camera}")
    print(f"Tag in camera frame coordinates: {tag_in_camera_frame}")
    
    # Calculate range and bearing in camera frame (OLD method)
    dx_cam, dy_cam, dz_cam = tag_in_camera_frame
    old_range = np.sqrt(dx_cam**2 + dy_cam**2)
    old_bearing = np.arctan2(dy_cam, dx_cam)
    
    # Calculate range and bearing in robot frame (NEW method)
    dx_robot, dy_robot = tag_position_world[0], tag_position_world[1]
    new_range = np.sqrt(dx_robot**2 + dy_robot**2)
    new_bearing = np.arctan2(dy_robot, dx_robot)
    
    print(f"\n=== MEASUREMENT COMPARISON ===")
    print(f"OLD method (direct camera coords):")
    print(f"  Camera coords: ({dx_cam:.3f}, {dy_cam:.3f}, {dz_cam:.3f})")
    print(f"  Range: {old_range:.3f}m, Bearing: {np.degrees(old_bearing):.1f}°")
    
    print(f"NEW method (transformed to base_link):")
    print(f"  Robot coords: ({dx_robot:.3f}, {dy_robot:.3f})")
    print(f"  Range: {new_range:.3f}m, Bearing: {np.degrees(new_bearing):.1f}°")
    
    print(f"\nBearing difference: {np.degrees(new_bearing - old_bearing):.1f}°")
    print(f"Range difference: {new_range - old_range:.3f}m")
    
    # Test the actual TF transformation that would be used
    print(f"\n=== TF TRANSFORMATION TEST ===")
    
    # This simulates what tf_buffer.transform() would do
    # Create homogeneous transformation matrix
    T_base_to_camera = np.eye(4)
    T_base_to_camera[:3, :3] = rotation_matrix
    T_base_to_camera[:3, 3] = camera_translation
    
    # Tag in camera frame as a homogeneous point
    tag_camera_homogeneous = np.array([dx_cam, dy_cam, dz_cam, 1])
    
    # Transform back to base_link
    tag_base_homogeneous = T_base_to_camera @ tag_camera_homogeneous
    tag_base_from_tf = tag_base_homogeneous[:3]
    
    print(f"TF transformation result: {tag_base_from_tf}")
    print(f"Expected base_link coords: {tag_position_world}")
    print(f"Transformation error: {np.linalg.norm(tag_base_from_tf - tag_position_world):.6f}m")
    
    # Test multiple scenarios
    print(f"\n=== MULTIPLE SCENARIO TEST ===")
    scenarios = [
        ("Front-center", [1.0, 0.0, 0.0]),
        ("Front-right", [1.0, 0.5, 0.0]),
        ("Front-left", [1.0, -0.5, 0.0]),
        ("Right side", [0.0, 1.0, 0.0]),
        ("Behind-right", [-1.0, 0.5, 0.0])
    ]
    
    for name, pos in scenarios:
        tag_pos = np.array(pos)
        tag_rel = tag_pos - camera_translation
        tag_cam = rotation_matrix.T @ tag_rel
        
        # OLD vs NEW bearings
        old_bear = np.degrees(np.arctan2(tag_cam[1], tag_cam[0]))
        new_bear = np.degrees(np.arctan2(tag_pos[1], tag_pos[0]))
        
        print(f"{name:12}: OLD={old_bear:6.1f}°, NEW={new_bear:6.1f}°, diff={new_bear-old_bear:6.1f}°")

if __name__ == "__main__":
    test_coordinate_transformation()
