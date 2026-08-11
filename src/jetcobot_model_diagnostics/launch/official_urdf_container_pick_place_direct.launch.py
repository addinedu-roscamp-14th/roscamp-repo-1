#!/usr/bin/env python3

"""Direct pick/place using official URDF TF and official 6_Link hand-eye."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_paths_and_make_official_rsp(context):
    camera_info = LaunchConfiguration("camera_info_url").perform(context)
    if "://" not in camera_info:
        camera_info = Path(camera_info).expanduser().resolve().as_uri()

    urdf_path = Path(
        LaunchConfiguration("official_urdf_path").perform(context)
    ).expanduser()
    if not urdf_path.is_file():
        raise RuntimeError(f"Official URDF not found: {urdf_path}")

    return [
        SetLaunchConfiguration("resolved_camera_info_url", camera_info),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="official_direct_robot_state_publisher",
            parameters=[{
                "robot_description": urdf_path.read_text(encoding="utf-8"),
            }],
            remappings=[("joint_states", "/arm/joint_states")],
            output="screen",
        ),
    ]


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("jetcobot_model_diagnostics")
    )
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
        DeclareLaunchArgument("speed", default_value="5"),
        DeclareLaunchArgument("video_device", default_value="/dev/video2"),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value="config/arm/gripper_camera_info.yaml",
        ),
        DeclareLaunchArgument(
            "camera_frame_id",
            default_value="arm/gripper_camera_optical_frame",
        ),
        DeclareLaunchArgument("pick_marker_id", default_value="2"),
        DeclareLaunchArgument("place_marker_id", default_value="8"),
        DeclareLaunchArgument("marker_size_m", default_value="0.015"),
        DeclareLaunchArgument("dictionary", default_value="DICT_5X5_50"),
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
        DeclareLaunchArgument(
            "params_file",
            default_value=str(
                package_share
                / "config"
                / "official_urdf_container_pick_place_direct.yaml"
            ),
        ),
    ]

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        namespace="arm/gripper_camera",
        name="camera",
        output="screen",
        parameters=[{
            "video_device": LaunchConfiguration("video_device"),
            "image_size": [640, 480],
            "time_per_frame": [1, 10],
            "pixel_format": "YUYV",
            "output_encoding": "rgb8",
            "camera_frame_id": LaunchConfiguration("camera_frame_id"),
            "camera_info_url": LaunchConfiguration(
                "resolved_camera_info_url"
            ),
        }],
    )

    detector = Node(
        package="arm",
        executable="dual_aruco_pose_publisher",
        name="dual_aruco_pose_publisher",
        namespace="arm",
        output="screen",
        parameters=[{
            "camera_frame_id": LaunchConfiguration("camera_frame_id"),
            "pick_marker_id": ParameterValue(
                LaunchConfiguration("pick_marker_id"), value_type=int
            ),
            "place_marker_id": ParameterValue(
                LaunchConfiguration("place_marker_id"), value_type=int
            ),
            "pick_marker_frame": "arm/pick_marker",
            "place_marker_frame": "arm/place_marker",
            "marker_size_m": ParameterValue(
                LaunchConfiguration("marker_size_m"), value_type=float
            ),
            "dictionary": LaunchConfiguration("dictionary"),
            "use_node_time_for_pose": ParameterValue(
                LaunchConfiguration("use_node_time_for_pose"),
                value_type=bool,
            ),
        }],
    )

    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(easy_handeye_share / "launch" / "publish.launch.py")
        ),
        launch_arguments={
            "name": LaunchConfiguration("calibration_name"),
        }.items(),
    )

    coordinator = Node(
        package="arm",
        executable="container_pick_place_coordinator",
        name="container_pick_place_coordinator",
        namespace="arm",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "base_frame": "base_link",
                "execute_motion": True,
                "motion_backend": "direct",
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": ParameterValue(
                    LaunchConfiguration("baud_rate"), value_type=int
                ),
                "speed": ParameterValue(
                    LaunchConfiguration("speed"), value_type=int
                ),
            },
        ],
    )

    return LaunchDescription(
        arguments
        + [
            OpaqueFunction(function=resolve_paths_and_make_official_rsp),
            SetEnvironmentVariable(
                "EASY_HANDEYE2_CALIBRATIONS_DIRECTORY",
                LaunchConfiguration("calibration_directory"),
            ),
            camera,
            handeye,
            TimerAction(period=1.5, actions=[detector]),
            coordinator,
        ]
    )
