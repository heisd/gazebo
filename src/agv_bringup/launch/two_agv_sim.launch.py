#!/usr/bin/env python3

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PARKING_POSES = {
    'agv_01': (10.8, 4.0, 3.1416),
    'agv_02': (10.8, -4.0, 3.1416),
}


def get_robot_description(namespace, prefix):
    desc_dir = get_package_share_directory('agv_description')
    urdf = os.path.join(desc_dir, 'urdf', 'agv_robot.urdf.xacro')
    result = subprocess.run(
        [
            'xacro',
            urdf,
            f'namespace:={namespace}',
            f'prefix:={prefix}',
            'enable_ros2_control:=false',
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def make_robot(namespace, prefix, x, y, yaw, z='0.12'):
    robot_desc = get_robot_description(namespace, prefix)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        respawn=True,
        respawn_delay=2.0,
        output='screen',
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        respawn=True,
        respawn_delay=2.0,
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', f'/{namespace}/robot_description',
            '-entity', namespace,
            '-x', str(x),
            '-y', str(y),
            '-Y', str(yaw),
            '-z', z,
        ],
        output='screen',
    )

    return rsp, jsp, spawn


def generate_launch_description():
    gaz_dir = get_package_share_directory('agv_gazebo')
    world = os.path.join(gaz_dir, 'worlds', 'warehouse.world')

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world,
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen',
    )

    agv_01_rsp, agv_01_jsp, agv_01_spawn = make_robot(
        'agv_01', 'agv_01_', *PARKING_POSES['agv_01'])
    agv_02_rsp, agv_02_jsp, agv_02_spawn = make_robot(
        'agv_02', 'agv_02_', *PARKING_POSES['agv_02'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        LogInfo(msg='[AGV] Starting two-AGV Gazebo simulation...'),
        gzserver,
        gzclient,
        agv_01_rsp,
        agv_02_rsp,
        agv_01_jsp,
        agv_02_jsp,
        TimerAction(period=3.0, actions=[agv_01_spawn, agv_02_spawn]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
