#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for HW2 sequence navigation.
    Starts two nodes:
    1. sequence_navigation - Time-based sequence control with AprilTag corrections
    2. motor_control - Communicates with robot hardware via serial
    Note: sequence_navigation bypasses velocity_mapping and sends motor commands directly
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Sequence Navigation Node (time-based control)
    sequence_navigation_node = Node(
        package='apriltag_sequence_navigation',
        executable='sequence_navigation',
        name='sequence_navigation_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Motor Control Node (for sending commands to robot)
    motor_control_node = Node(
        package='apriltag_sequence_navigation',
        executable='motor_control',
        name='motor_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        sequence_navigation_node,
        motor_control_node
    ])
