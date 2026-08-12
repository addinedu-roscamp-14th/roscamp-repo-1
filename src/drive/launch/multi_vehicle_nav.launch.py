"""Start one namespaced Nav2 stack with vehicle-specific TF frames."""

from pathlib import Path
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _rewrite_frames(value, vehicle_id):
    if isinstance(value, dict):
        rewritten = {}
        for key, child in value.items():
            child = _rewrite_frames(child, vehicle_id)
            if key == 'base_frame_id':
                child = f'{vehicle_id}/base_footprint'
            elif key == 'odom_frame_id':
                child = f'{vehicle_id}/odom'
            elif key == 'robot_base_frame':
                suffix = 'base_link' if child == 'base_link' else 'base_footprint'
                child = f'{vehicle_id}/{suffix}'
            elif key in ('local_frame', 'global_frame') and child == 'odom':
                child = f'{vehicle_id}/odom'
            rewritten[key] = child
        return rewritten
    if isinstance(value, list):
        return [_rewrite_frames(item, vehicle_id) for item in value]
    return value


def _rewrite_keepout_topics(params, vehicle_id):
    """Use absolute filter topics so nested costmap nodes resolve them correctly."""
    filter_info_topic = f'/{vehicle_id}/costmap_filter_info'
    mask_topic = f'/{vehicle_id}/keepout_filter_mask'

    for costmap_name in ('local_costmap', 'global_costmap'):
        costmap = params[costmap_name][costmap_name]['ros__parameters']
        costmap['keepout_filter']['filter_info_topic'] = filter_info_topic

    mask_server = params['filter_mask_server']['ros__parameters']
    mask_server['topic_name'] = mask_topic

    info_server = params['costmap_filter_info_server']['ros__parameters']
    info_server['filter_info_topic'] = filter_info_topic
    info_server['mask_topic'] = mask_topic


def _rewrite_other_robot_topics(params, vehicle_id):
    """Keep peer-obstacle topics at the vehicle namespace root."""
    local_costmap = params['local_costmap']['local_costmap']['ros__parameters']
    layer = local_costmap['other_robot_layer']
    layer['other_robot']['topic'] = f'/{vehicle_id}/other_robot_obstacle'
    layer['other_robot_clear']['topic'] = (
        f'/{vehicle_id}/other_robot_obstacle_clear'
    )


def _launch_nav2(context):
    vehicle_id = LaunchConfiguration('vehicle_id').perform(context).strip('/')
    if vehicle_id not in ('agv1', 'agv2'):
        raise ValueError('vehicle_id must be agv1 or agv2')

    source = Path(LaunchConfiguration('params_file').perform(context))
    with source.open('r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    params = _rewrite_frames(params, vehicle_id)
    _rewrite_keepout_topics(params, vehicle_id)
    _rewrite_other_robot_topics(params, vehicle_id)
    # ROS parameter files match fully-qualified node names. Nesting the file
    # under the vehicle namespace makes every Nav2 block apply to /agvX/*.
    params.pop('/**', None)
    params = {vehicle_id: params}
    generated = Path(tempfile.gettempdir()) / f'porter_nav2_{vehicle_id}.yaml'
    with generated.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, sort_keys=False)

    other_vehicle_id = 'agv2' if vehicle_id == 'agv1' else 'agv1'
    return [
        Node(
            package='drive',
            executable='amcl_pose_heartbeat',
            name='amcl_pose_heartbeat',
            namespace=vehicle_id,
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_other_robot_obstacle')
            ),
            parameters=[{
                'input_topic': f'/{vehicle_id}/amcl_pose',
                'output_topic': 'shared_amcl_pose',
                'frame_id': 'map',
                'publish_rate': LaunchConfiguration(
                    'shared_pose_publish_rate'
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='drive',
            executable='other_robot_obstacle',
            name='other_robot_obstacle',
            namespace=vehicle_id,
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_other_robot_obstacle')
            ),
            parameters=[{
                'other_pose_topic': f'/{other_vehicle_id}/shared_amcl_pose',
                'obstacle_topic': 'other_robot_obstacle',
                'clearing_topic': 'other_robot_obstacle_clear',
                'frame_id': 'map',
                'cloud_frame_id': f'{vehicle_id}/base_footprint',
                'obstacle_radius': LaunchConfiguration(
                    'other_robot_obstacle_radius'
                ),
                'point_spacing': LaunchConfiguration(
                    'other_robot_point_spacing'
                ),
                'publish_rate': LaunchConfiguration(
                    'other_robot_publish_rate'
                ),
                'other_robot_pose_timeout': LaunchConfiguration(
                    'other_robot_pose_timeout'
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('drive'),
                    'launch',
                    'bringup_launch.xml',
                ])
            ),
            launch_arguments={
                'namespace': vehicle_id,
                'workspace': LaunchConfiguration('workspace'),
                'map': LaunchConfiguration('map'),
                'keepout_mask': LaunchConfiguration('keepout_mask'),
                'params_file': str(generated),
                'container_name': 'nav2_container',
                'container_target': f'/{vehicle_id}/nav2_container',
                'cmd_vel_output_topic': 'cmd_vel_safe_input',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_composition': LaunchConfiguration('use_composition'),
                'start_navigation': LaunchConfiguration('start_navigation'),
            }.items(),
        ),
    ]


def generate_launch_description():
    workspace = LaunchConfiguration('workspace')
    vehicle_id = LaunchConfiguration('vehicle_id')
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_id',
            description='Vehicle namespace: agv1 or agv2',
            choices=['agv1', 'agv2'],
        ),
        DeclareLaunchArgument(
            'workspace',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'poter_ws',
            ]),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                workspace, 'config', 'SLAM', 'current_map.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=PathJoinSubstitution([
                workspace, 'config', 'SLAM', 'keepout_mask.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('drive'), 'params', 'nav2_params.yaml',
            ]),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('start_navigation', default_value='True'),
        DeclareLaunchArgument(
            'start_other_robot_obstacle',
            default_value='True',
            description=(
                'Publish the peer AGV AMCL pose into this local costmap'
            ),
        ),
        DeclareLaunchArgument(
            'other_robot_obstacle_radius',
            default_value='0.13',
            description='Virtual peer obstacle radius in metres',
        ),
        DeclareLaunchArgument(
            'other_robot_point_spacing',
            default_value='0.02',
            description='PointCloud2 grid spacing in metres',
        ),
        DeclareLaunchArgument(
            'shared_pose_publish_rate',
            default_value='10.0',
            description='AMCL pose heartbeat publication rate in Hz',
        ),
        DeclareLaunchArgument(
            'other_robot_publish_rate',
            default_value='10.0',
            description='Virtual obstacle publication rate in Hz',
        ),
        DeclareLaunchArgument(
            'other_robot_pose_timeout',
            default_value='1.0',
            description='Seconds before a missing peer pose is cleared',
        ),
        DeclareLaunchArgument(
            'start_parking_supervisor',
            default_value='True',
            description='Run the namespaced auto-parking action server',
        ),
        DeclareLaunchArgument(
            'parking_spots_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('drive'), 'params', 'parking_spots.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'parking_supervisor_start_delay',
            default_value='15.0',
            description=(
                'Seconds to wait after this launch group starts before '
                'starting parking_new, so it does not start while the '
                'Nav2 composable-node burst is still loading'
            ),
        ),
        OpaqueFunction(function=_launch_nav2),
        TimerAction(
            # Starting alongside the ~12 Nav2 composable nodes overloads the
            # Pi enough that parking_new dies during rclpy/DDS init with no
            # captured traceback. Let the composable-node burst finish first.
            period=LaunchConfiguration('parking_supervisor_start_delay'),
            condition=IfCondition(LaunchConfiguration('start_parking_supervisor')),
            actions=[
                Node(
                    package='drive',
                    executable='parking_new',
                    name='parking_supervisor',
                    namespace=vehicle_id,
                    output='screen',
                    parameters=[{
                        'parking_spots_yaml': LaunchConfiguration(
                            'parking_spots_yaml'
                        ),
                        'cmd_vel_topic': 'cmd_vel_safe_input',
                    }],
                ),
            ],
        ),
    ])
