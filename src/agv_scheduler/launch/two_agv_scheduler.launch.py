#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scheduler_dir = get_package_share_directory('agv_scheduler')
    default_params = os.path.join(
        scheduler_dir, 'config', 'two_agv_scheduler.yaml')
    default_layout = os.path.join(
        scheduler_dir, 'config', 'warehouse_layout.yaml')
    params_file = LaunchConfiguration('params_file')
    layout_file = LaunchConfiguration('shelf_layout_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    scheduler = Node(
        package='agv_scheduler',
        executable='scheduler_node',
        name='agv_scheduler',
        parameters=[
            params_file,
            {
                'shelf_layout_file': layout_file,
                'use_sim_time': use_sim_time,
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'shelf_layout_file', default_value=default_layout),
        scheduler,
    ])
