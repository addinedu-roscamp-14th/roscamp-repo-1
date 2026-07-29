"""Start one namespaced Nav2 stack with vehicle-specific TF frames."""

from pathlib import Path
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
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


def _launch_nav2(context):
    vehicle_id = LaunchConfiguration('vehicle_id').perform(context).strip('/')
    if vehicle_id not in ('agv1', 'agv2'):
        raise ValueError('vehicle_id must be agv1 or agv2')

    source = Path(LaunchConfiguration('params_file').perform(context))
    with source.open('r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    params = _rewrite_frames(params, vehicle_id)
    generated = Path(tempfile.gettempdir()) / f'porter_nav2_{vehicle_id}.yaml'
    with generated.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, sort_keys=False)

    return [
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
                'container_name': f'{vehicle_id}_nav2_container',
                'cmd_vel_output_topic': 'cmd_vel_safe_input',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_composition': LaunchConfiguration('use_composition'),
                'start_navigation': LaunchConfiguration('start_navigation'),
            }.items(),
        )
    ]


def generate_launch_description():
    workspace = LaunchConfiguration('workspace')
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
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('start_navigation', default_value='True'),
        OpaqueFunction(function=_launch_nav2),
    ])
