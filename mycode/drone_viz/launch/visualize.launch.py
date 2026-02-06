"""
Launch file for drone visualization with proper startup sequencing.
Ensures all publishers are ready before RViz subscribes.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_frame = LaunchConfiguration('world_frame')

    # Get package paths
    drone_description_share = get_package_share_directory('drone_description')
    drone_viz_share = get_package_share_directory('drone_viz')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'world_frame',
            default_value='optitrack',
            description='OptiTrack world frame'
        ),

        # OptiTrack TF Publisher
        Node(
            package='optitrack',
            executable='talker',
            name='optitrack_publisher',
            output='screen'
        ),

        # Robot State Publisher (publishes URDF transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    os.path.join(drone_description_share, 'urdf', 'X500V2.urdf.xacro')
                ])
            }]
        ),

        # Arena corner columns (purple markers)
        Node(
            package='drone_description',
            executable='arena_markers_node',
            name='arena_markers',
            output='screen',
            parameters=[{
                'frame_id': world_frame,
                'column_width': 0.1,
                'column_height': 2.92,
                'x_back': -1.69,
                'x_front': 1.69,
                'y_left': 2.15,
                'y_right': -3.27,
            }]
        ),

        # RViz - delayed by 2 seconds to let publishers initialize
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', os.path.join(drone_viz_share, 'config', 'drone_viz.rviz')]
                )
            ]
        ),
    ])
