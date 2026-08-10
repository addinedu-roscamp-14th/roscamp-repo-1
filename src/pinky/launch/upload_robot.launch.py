import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, PythonExpression

def generate_launch_description():
    ld = LaunchDescription()

    namespace_arg = DeclareLaunchArgument("namespace", default_value="")
    is_sim = DeclareLaunchArgument("is_sim", default_value="false")
    cam_tilt_deg = DeclareLaunchArgument("cam_tilt_deg", default_value="0")
    start_joint_state_publisher = DeclareLaunchArgument(
        "start_joint_state_publisher",
        default_value="true",
    )

    namespace = PythonExpression([
        "'", LaunchConfiguration('namespace'), "' + ('/' if '", LaunchConfiguration('namespace'), "' != '' else '')"
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'ignore_timestamp': False,
            "use_sim_time": LaunchConfiguration('is_sim'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        get_package_share_directory('pinky'),
                        'urdf/robot.urdf.xacro',
                    ]),
                    ' namespace:=', namespace,
                    ' is_sim:=', LaunchConfiguration('is_sim'),
                    ' cam_tilt_deg:=', LaunchConfiguration('cam_tilt_deg')
                ]),
            # Xacro prefixes joint names; robot_state_publisher prefixes links.
            'frame_prefix': [namespace],
        }]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(
            LaunchConfiguration('start_joint_state_publisher')
        ),
        parameters=[{
            "source_list": ['joint_states'],
            "rate": 20.0,
            "use_sim_time": LaunchConfiguration('is_sim'),
        }],
        remappings=[
            ('/robot_descrption', 'robot_descrpition'),
        ],
        output='screen'
    )

    ld.add_action(namespace_arg)
    ld.add_action(is_sim)
    ld.add_action(cam_tilt_deg)
    ld.add_action(start_joint_state_publisher)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)

    return ld
