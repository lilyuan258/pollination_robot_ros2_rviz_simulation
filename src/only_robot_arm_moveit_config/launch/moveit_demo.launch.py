import os
import math
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
    rviz_config = os.path.join(
        get_package_share_directory("only_robot_arm_moveit_config"),
        "launch",
        "moveit.rviz",
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("only_robot_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

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
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "only_robot_arm_moveit_config", "config/joint_limits.yaml"
        )
    }
    ompl_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "start_state_max_bounds_error": 0.1,
            **load_yaml("only_robot_arm_moveit_config", "config/ompl_planning.yaml"),
        },
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    trajectory_execution = {
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
    }

    flower_dx_world = LaunchConfiguration("flower_dx_world")
    flower_dy_world = LaunchConfiguration("flower_dy_world")
    flower_center_z = LaunchConfiguration("flower_center_z")
    use_rviz = LaunchConfiguration("use_rviz")

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0.89",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            str(math.pi),
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_pipeline,
            trajectory_execution,
            planning_scene_monitor_parameters,
        ],
    )

    flower_marker_node = Node(
        package="only_robot_arm",
        executable="watermelon_flower_marker.py",
        name="watermelon_flower_marker",
        output="screen",
        parameters=[
            {
                "world_frame": "world",
                "anchor_frame": "link_1",
                "dx_world": flower_dx_world,
                "dy_world": flower_dy_world,
                "flower_center_z": flower_center_z,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_pipeline,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/ros_logs"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            DeclareLaunchArgument("ros_domain_id", default_value="0"),
            DeclareLaunchArgument("flower_dx_world", default_value="0.1"),
            DeclareLaunchArgument("flower_dy_world", default_value="-0.8"),
            DeclareLaunchArgument("flower_center_z", default_value="0.1"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            flower_marker_node,
            rviz_node,
        ]
    )
