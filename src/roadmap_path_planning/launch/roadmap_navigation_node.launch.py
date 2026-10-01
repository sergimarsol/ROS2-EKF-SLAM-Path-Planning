from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch file for roadmap_path_planning package.
    Starts nodes in sequence:
    0. Process cleanup - kill existing processes
    1. robot_vision_camera - camera driver
    2. apriltag_ros - AprilTag detection
    3. [5s delay]
    4. motor_control - communicates with robot hardware via serial
    5. velocity_mapping - converts Twist commands to motor commands
    6. camera_tf - static tf_node
    7. [10s delay total]
    8. roadmap_navigation_node - waypoint navigation around obstacle given a planned path
    """
    
    # Process cleanup - kill any existing relevant processes
    cleanup_processes = ExecuteProcess(
        cmd=['bash', '-c', 
             'sudo pkill -f qtiqmmfsrc || true; '
             'sudo pkill -f gstreamer || true; '
             'sudo pkill -f robot_vision || true; '
             'killall -9 robot_vision_camera_node foxglove_bridge || true; '
             'sleep 3'],
        output='screen'
    )
    
    camera_pkg_path = FindPackageShare(package='robot_vision_camera').find('robot_vision_camera')
    apriltag_pkg_path = FindPackageShare(package='apriltag_ros').find('apriltag_ros')
    pkg_path = FindPackageShare(package='roadmap_path_planning').find('roadmap_path_planning')
    
    camera_launch_file = os.path.join(camera_pkg_path, 'launch', 'robot_vision_camera.launch.py')
    apriltag_launch_file = os.path.join(apriltag_pkg_path, 'launch', 'apriltag_launch.py')
    
    # Start camera with delay to ensure cleanup is complete
    camera_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch_file),
                launch_arguments={
                    'image_rectify': 'true',
                    'camera_parameter_path': os.path.expanduser('~/ros2_ws/src/robot_vision/config/camera_parameter.yaml')
                }.items()
            )
        ]
    )
    
    # Start AprilTag detection with parameters
    apriltag_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(apriltag_launch_file),
                launch_arguments={
                    'image_topic': '/camera/image_raw',
                    'camera_info_topic': '/camera/camera_info',
                    'config_file': 'tags_standard41h12.yaml'
                }.items()
            )
        ]
    )
    
    motor_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='roadmap_path_planning',
                executable='motor_control',
                name='motor_control',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    velocity_mapping = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='roadmap_path_planning',
                executable='velocity_mapping',
                name='velocity_mapping',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    camera_tf = Node(
        package='roadmap_path_planning',
        executable='camera_tf',
        name='camera_tf',
        output='screen',
        emulate_tty=True,
    )
    
    roadmap_navigation_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='roadmap_path_planning',
                executable='roadmap_navigation_node',
                name='roadmap_navigation_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    return LaunchDescription([
        cleanup_processes,
        camera_launch,
        apriltag_launch,
        motor_controller,
        velocity_mapping,
        camera_tf,
        roadmap_navigation_node,
    ])
