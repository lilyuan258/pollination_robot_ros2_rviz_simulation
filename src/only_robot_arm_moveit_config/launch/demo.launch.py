import math
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _normalize_for_parameters(value):
    if isinstance(value, dict):
        return {k: _normalize_for_parameters(v) for k, v in value.items()}
    if isinstance(value, tuple):
        return [_normalize_for_parameters(v) for v in value]
    if isinstance(value, list):
        return [_normalize_for_parameters(v) for v in value]
    return value


def _resolve_base_cw(context, *args, **kwargs):
    base_cw = LaunchConfiguration("base_clockwise_delta_rad").perform(context)

    # 用户如果显式传了，就尊重用户输入
    if base_cw != "":
        return []

    dy_world = float(LaunchConfiguration("dy_world").perform(context))

    if dy_world < 0.0:
        value = str(-math.pi / 2.0)
    elif dy_world > 0.0:
        value = str(math.pi / 2.0)
    else:
        value = "0.0"

    return [SetLaunchConfiguration("base_clockwise_delta_rad", value)]


def generate_launch_description():
    arm_pkg = get_package_share_directory("only_robot_arm")
    moveit_pkg = get_package_share_directory("only_robot_arm_moveit_config")
    urdf_path = os.path.join(arm_pkg, "urdf", "only_robot_arm.urdf")
    rviz_path = os.path.join(moveit_pkg, "launch", "moveit.rviz")
    env_visual_config = os.path.join(arm_pkg, "config", "environment_visual.yaml")
    initial_positions_path = os.path.join(moveit_pkg, "config", "initial_positions.yaml")

    contracted_joint_names = []
    contracted_joint_values = []
    if os.path.exists(initial_positions_path):
        with open(initial_positions_path, "r", encoding="utf-8") as f:
            initial_positions_data = yaml.safe_load(f) or {}
        contracted_map = initial_positions_data.get("initial_positions", {})
        contracted_joint_names = list(contracted_map.keys())
        contracted_joint_values = [float(contracted_map[name]) for name in contracted_joint_names]

    moveit_config = (
        MoveItConfigsBuilder("only_robot_arm", package_name="only_robot_arm_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/only_robot_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    run_cycle_arg = DeclareLaunchArgument("run_cycle", default_value="true")
    single_cycle_snapshot_mode_arg = DeclareLaunchArgument("single_cycle_snapshot_mode", default_value="true")
    dx_world_arg = DeclareLaunchArgument("dx_world", default_value="0.1")
    dy_world_arg = DeclareLaunchArgument("dy_world", default_value="-0.8")
    flower_z_arg = DeclareLaunchArgument("flower_center_z", default_value="0.1")

    base_cw_arg = DeclareLaunchArgument(
        "base_clockwise_delta_rad",
        default_value="",
        description="Override clockwise rotation around joint_1 axis. Leave empty to auto-derive from dy_world.",
    )

    resolve_base_cw = OpaqueFunction(function=_resolve_base_cw)

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=["0", "0", "0.89", "0", "0", str(math.pi), "world", "base_link"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    marker_node = Node(
        package="only_robot_arm",
        executable="watermelon_flower_marker.py",
        name="watermelon_flower_marker",
        output="screen",
        parameters=[
            {
                "world_frame": "world",
                "anchor_frame": "link_1",
                "dx_world": ParameterValue(LaunchConfiguration("dx_world"), value_type=float),
                "dy_world": ParameterValue(LaunchConfiguration("dy_world"), value_type=float),
                "flower_center_z": ParameterValue(LaunchConfiguration("flower_center_z"), value_type=float),
            }
        ],
    )

    environment_visual_node = Node(
        package="only_robot_arm",
        executable="environment_visual_marker.py",
        name="environment_visual_marker",
        output="screen",
        parameters=[env_visual_config],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[("joint_states", "/joint_states")],
        parameters=[
            _normalize_for_parameters(moveit_config.to_dict()),
            {
                "allow_trajectory_execution": False,
                "moveit_manage_controllers": False,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    pollination_cycle_node = Node(
        package="only_robot_arm",
        executable="pollination_cycle_node",
        name="pollination_cycle_node",
        output="screen",
        remappings=[("joint_states", "/joint_states")],
        parameters=[
            {
                "world_frame": "world",
                "base_frame": "base_link",
                "virtual_joint_name": "world_joint",
                "anchor_frame": "link_1",
                "tip_frame": "pollination_tip_link",
                "joint6_frame": "link_6",
                "planning_group": "arm",
                "dx_world": ParameterValue(LaunchConfiguration("dx_world"), value_type=float),
                "dy_world": ParameterValue(LaunchConfiguration("dy_world"), value_type=float),
                "flower_center_z": ParameterValue(LaunchConfiguration("flower_center_z"), value_type=float),
                "base_clockwise_delta_rad": ParameterValue(
                    LaunchConfiguration("base_clockwise_delta_rad"), value_type=float
                ),
                "single_cycle_snapshot_mode": ParameterValue(
                    LaunchConfiguration("single_cycle_snapshot_mode"), value_type=bool
                ),
                "contracted_joint_names": contracted_joint_names,
                "contracted_joint_values": contracted_joint_values,
            }
        ],
        condition=IfCondition(LaunchConfiguration("run_cycle")),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            run_cycle_arg,
            single_cycle_snapshot_mode_arg,
            dx_world_arg,
            dy_world_arg,
            flower_z_arg,
            base_cw_arg,
            resolve_base_cw,
            static_tf,
            robot_state_publisher,
            marker_node,
            environment_visual_node,
            move_group_node,
            rviz_node,
            pollination_cycle_node,
        ]
    )
