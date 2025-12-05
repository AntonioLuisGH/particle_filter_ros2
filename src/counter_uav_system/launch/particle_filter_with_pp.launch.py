import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='drone_world',
        description='The name of the global fixed frame'
    )

    # 1. Adapter Node - NOW WITH MARKER MODE ENABLED
    adapter_node = Node(
        package='counter_uav_system',
        executable='bbox_adapter_node', 
        name='bbox_adapter',
        output='screen',
        parameters=[{
            'use_markers': True,   # <--- ENABLE THIS to fix the "Wait for Data" issue
            'input_topic': '/pointpillars_bbox'
        }]
    )

    # 2. Particle Filter
    particle_filter_node = Node(
        package='counter_uav_system',
        executable='particle_filter_node',
        name='particle_filter_node',
        output='screen',
        parameters=[{
            'global_frame': LaunchConfiguration('global_frame')
        }]
    )

    # 3. Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', LaunchConfiguration('global_frame'), 'sensor_frame']
    )

    return LaunchDescription([
        global_frame_arg,
        adapter_node,
        particle_filter_node,
        static_tf
    ])