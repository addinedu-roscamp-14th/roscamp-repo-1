#!/usr/bin/env python3

"""Calibrate eye-in-hand using the official URDF and 6_Link as effector."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


# The official URDF has no TCP. This semantic model deliberately ends the
# planning group at the official flange frame, 6_Link.
OFFICIAL_SRDF = """<?xml version="1.0"?>
<robot name="jetcobot">
  <group name="arm_group">
    <chain base_link="base_link" tip_link="6_Link"/>
  </group>
  <group_state name="home" group="arm_group">
    <joint name="1_Joint" value="0"/>
    <joint name="2_Joint" value="0"/>
    <joint name="3_Joint" value="0"/>
    <joint name="4_Joint" value="0"/>
    <joint name="5_Joint" value="0"/>
    <joint name="6_Joint" value="0"/>
  </group_state>
  <virtual_joint name="world_joint" type="fixed"
                 parent_frame="world" child_link="dummy"/>
  <disable_collisions link1="base_link" link2="1_Link" reason="Adjacent"/>
  <disable_collisions link1="1_Link" link2="2_Link" reason="Adjacent"/>
  <disable_collisions link1="2_Link" link2="3_Link" reason="Adjacent"/>
  <disable_collisions link1="3_Link" link2="4_Link" reason="Adjacent"/>
  <disable_collisions link1="4_Link" link2="5_Link" reason="Adjacent"/>
  <disable_collisions link1="5_Link" link2="6_Link" reason="Adjacent"/>
  <disable_collisions link1="6_Link" link2="camera_link" reason="Adjacent"/>
  <disable_collisions link1="6_Link" link2="jiazhua_Link" reason="Adjacent"/>
  <disable_collisions link1="camera_link" link2="jiazhua_Link" reason="Never"/>
</robot>
"""


def make_official_moveit_nodes(context):
    """Replace only MoveIt's robot model; retain planner/controller settings."""
    urdf_path = Path(
        LaunchConfiguration("official_urdf_path").perform(context)
    ).expanduser()
    if not urdf_path.is_file():
        raise RuntimeError(f"Official URDF not found: {urdf_path}")

    moveit_config = (
        MoveItConfigsBuilder(
            "jetcobot",
            package_name="jetcobot_moveit_config",
        )
        .to_moveit_configs()
    )
    moveit_config.robot_description = {
        "robot_description": urdf_path.read_text(encoding="utf-8")
    }
    moveit_config.robot_description_semantic = {
        "robot_description_semantic": OFFICIAL_SRDF
    }

    move_group_parameters = moveit_config.to_dict()
    move_group_parameters.update(
        {
            "allow_trajectory_execution": True,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }
    )

    rviz_config = (
        Path(get_package_share_directory("jetcobot_moveit_config"))
        / "config"
        / "moveit.rviz"
    )
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="official_world_to_dummy",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world",
                "--child-frame-id", "dummy",
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="official_urdf_robot_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            parameters=[move_group_parameters],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="official_urdf_calibration_rviz",
            arguments=["-d", str(rviz_config)],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    arm_share = Path(get_package_share_directory("arm"))
    easy_handeye_share = Path(get_package_share_directory("easy_handeye2"))

    arguments = [
        DeclareLaunchArgument(
            "official_urdf_path",
            default_value=(
                "/home/choe-gyu-seung/bizlink-Yahboom.jetcobot_ws/"
                "src/jetcobot_description/urdf/jetcobot.urdf"
            ),
        ),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="1000000"),
        DeclareLaunchArgument("trajectory_speed", default_value="15"),
        DeclareLaunchArgument("goal_correction_speed", default_value="15"),
        DeclareLaunchArgument("goal_tolerance_deg", default_value="2.5"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="15.0"),
        DeclareLaunchArgument("joint_state_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "validation_mode",
            default_value="false",
            description=(
                "Load the saved calibration instead of starting the "
                "calibration server and its temporary camera transform."
            ),
        ),
        DeclareLaunchArgument("video_device", default_value="/dev/video2"),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value="config/arm/gripper_camera_info.yaml",
        ),
        DeclareLaunchArgument(
            "camera_frame_id",
            default_value="arm/gripper_camera_optical_frame",
        ),
        DeclareLaunchArgument(
            "board_frame_id", default_value="arm/handeye_target"
        ),
        DeclareLaunchArgument("dictionary", default_value="DICT_4X4_50"),
        DeclareLaunchArgument("squares_x", default_value="5"),
        DeclareLaunchArgument("squares_y", default_value="5"),
        DeclareLaunchArgument("square_length_m", default_value="0.020"),
        DeclareLaunchArgument("marker_length_m", default_value="0.015"),
        DeclareLaunchArgument("legacy_pattern", default_value="true"),
        DeclareLaunchArgument("detection_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("opencv_num_threads", default_value="1"),
        DeclareLaunchArgument(
            "minimum_charuco_corners", default_value="6"
        ),
        DeclareLaunchArgument(
            "max_reprojection_error_px", default_value="3.0"
        ),
        DeclareLaunchArgument(
            "use_node_time_for_pose", default_value="true"
        ),
        DeclareLaunchArgument(
            "calibration_name",
            default_value="jetcobot_eye_in_hand_charuco_official_6link",
        ),
        DeclareLaunchArgument(
            "calibration_directory", default_value="config/arm"
        ),
    ]

    bridge = Node(
        package="arm",
        executable="jetcobot_trajectory_bridge",
        name="jetcobot_trajectory_bridge",
        namespace="arm",
        output="screen",
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": ParameterValue(
                    LaunchConfiguration("baud_rate"), value_type=int
                ),
                "speed": ParameterValue(
                    LaunchConfiguration("trajectory_speed"), value_type=int
                ),
                "goal_correction_speed": ParameterValue(
                    LaunchConfiguration("goal_correction_speed"),
                    value_type=int,
                ),
                "goal_tolerance_deg": ParameterValue(
                    LaunchConfiguration("goal_tolerance_deg"),
                    value_type=float,
                ),
                "goal_timeout_sec": ParameterValue(
                    LaunchConfiguration("goal_timeout_sec"), value_type=float
                ),
                "joint_state_rate_hz": ParameterValue(
                    LaunchConfiguration("joint_state_rate_hz"),
                    value_type=float,
                ),
                "joint_states_topic": "/joint_states",
                "additional_joint_states_topic": "/arm/joint_states",
            }
        ],
    )

    gripper_charuco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / "launch" / "gripper_charuco.launch.py")
        ),
        launch_arguments={
            "video_device": LaunchConfiguration("video_device"),
            "camera_info_url": LaunchConfiguration("camera_info_url"),
            "camera_frame_id": LaunchConfiguration("camera_frame_id"),
            "board_frame_id": LaunchConfiguration("board_frame_id"),
            "dictionary": LaunchConfiguration("dictionary"),
            "squares_x": LaunchConfiguration("squares_x"),
            "squares_y": LaunchConfiguration("squares_y"),
            "square_length_m": LaunchConfiguration("square_length_m"),
            "marker_length_m": LaunchConfiguration("marker_length_m"),
            "legacy_pattern": LaunchConfiguration("legacy_pattern"),
            "detection_rate_hz": LaunchConfiguration("detection_rate_hz"),
            "opencv_num_threads": LaunchConfiguration("opencv_num_threads"),
            "minimum_charuco_corners": LaunchConfiguration(
                "minimum_charuco_corners"
            ),
            "max_reprojection_error_px": LaunchConfiguration(
                "max_reprojection_error_px"
            ),
            "use_node_time_for_pose": LaunchConfiguration(
                "use_node_time_for_pose"
            ),
        }.items(),
    )

    easy_handeye_calibrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(easy_handeye_share / "launch" / "calibrate.launch.py")
        ),
        launch_arguments={
            "name": LaunchConfiguration("calibration_name"),
            "calibration_type": "eye_in_hand",
            "robot_base_frame": "base_link",
            "robot_effector_frame": "6_Link",
            "tracking_base_frame": LaunchConfiguration("camera_frame_id"),
            "tracking_marker_frame": LaunchConfiguration("board_frame_id"),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("validation_mode")),
    )

    saved_calibration_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(easy_handeye_share / "launch" / "publish.launch.py")
        ),
        launch_arguments={
            "name": LaunchConfiguration("calibration_name"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("validation_mode")),
    )

    return LaunchDescription(
        arguments
        + [
            SetEnvironmentVariable(
                "EASY_HANDEYE2_CALIBRATIONS_DIRECTORY",
                LaunchConfiguration("calibration_directory"),
            ),
            OpaqueFunction(function=make_official_moveit_nodes),
            bridge,
            gripper_charuco,
            easy_handeye_calibrator,
            saved_calibration_publisher,
        ]
    )
