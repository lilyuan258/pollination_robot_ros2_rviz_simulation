import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return file.read()


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    robot_description = {
        "robot_description": load_file("only_robot_arm", "urdf/only_robot_arm.urdf")
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "only_robot_arm_moveit_config", "config/only_robot_arm.srdf"
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "only_robot_arm_moveit_config", "config/kinematics.yaml"
        )
    }

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("only_robot_arm_moveit_config"), "launch", "moveit_demo.launch.py"]
            )
        ),
        launch_arguments={
            "flower_dx_world": LaunchConfiguration("flower_dx_world"),
            "flower_dy_world": LaunchConfiguration("flower_dy_world"),
            "flower_center_z": LaunchConfiguration("flower_center_z"),
            "ros_domain_id": ros_domain_id,
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    pollination_cycle = Node(
        package="only_robot_arm_pollination",
        executable="pollination_cycle_node",
        name="pollination_cycle_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("only_robot_arm_pollination"), "config", "pollination.yaml"]
            ),
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    delayed_cycle = TimerAction(
        period=5.0,
        actions=[pollination_cycle],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            DeclareLaunchArgument("ros_domain_id", default_value="0"),
            DeclareLaunchArgument("flower_dx_world", default_value="0.1"),
            DeclareLaunchArgument("flower_dy_world", default_value="-0.8"),
            DeclareLaunchArgument("flower_center_z", default_value="0.1"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            moveit_demo,
            delayed_cycle,
        ]
    )
