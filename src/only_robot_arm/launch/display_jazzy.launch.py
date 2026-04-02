from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import math

def generate_launch_description():
    pkg_dir = get_package_share_directory('only_robot_arm')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'only_robot_arm.urdf')
    rviz_config = os.path.join(pkg_dir, 'config', 'only_robot_arm.rviz')
    env_visual_config = os.path.join(pkg_dir, 'config', 'environment_visual.yaml')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            arguments=[
                '0', '0', '0.89',
                '0', '0', str(math.pi),
                'world', 'base_link'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='only_robot_arm',
            executable='watermelon_flower_marker.py',
            name='watermelon_flower_marker',
            output='screen',
            parameters=[{
                'world_frame': 'world',
                'anchor_frame': 'link_1'
            }]
        ),

        Node(
            package='only_robot_arm',
            executable='environment_visual_marker.py',
            name='environment_visual_marker',
            output='screen',
            parameters=[env_visual_config]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
