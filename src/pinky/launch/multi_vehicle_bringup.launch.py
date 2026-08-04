"""Namespaced Pinky hardware bringup for one physical AGV."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_id = LaunchConfiguration('vehicle_id')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    motor_serial_port = LaunchConfiguration('motor_serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_id',
            description='Required vehicle namespace: agv1 or agv2',
            choices=['agv1', 'agv2'],
        ),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument(
            'motor_serial_port',
            default_value='/dev/ttyAMA5',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('pinky'),
                    'launch',
                    'upload_robot.launch.py',
                ])
            ),
            launch_arguments={
                'namespace': vehicle_id,
                'is_sim': use_sim_time,
            }.items(),
        ),
        Node(
            package='pinky',
            executable='sllidar_node',
            namespace=vehicle_id,
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_serial_port,
                'serial_baudrate': 460800,
                'frame_id': ParameterValue(
                    [vehicle_id, '/rplidar_link'],
                    value_type=str,
                ),
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
            }],
        ),
        Node(
            package='pinky',
            executable='bringup',
            namespace=vehicle_id,
            name='pinky',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('pinky'),
                    'config',
                    'pinky_params.yaml',
                ]),
                {
                    'frame_prefix': ParameterValue(
                        vehicle_id,
                        value_type=str,
                    ),
                    'left_wheel_joint': ParameterValue(
                        [vehicle_id, '/l_wheel_joint'],
                        value_type=str,
                    ),
                    'right_wheel_joint': ParameterValue(
                        [vehicle_id, '/r_wheel_joint'],
                        value_type=str,
                    ),
                    'serial_port': motor_serial_port,
                },
            ],
        ),
        Node(
            package='pinky',
            executable='battery_publisher',
            namespace=vehicle_id,
            name='battery_publisher',
            output='screen',
        ),
        Node(
            package='pinky',
            executable='cmd_vel_safety_gate',
            namespace=vehicle_id,
            name='cmd_vel_safety_gate',
            output='screen',
        ),
    ])
