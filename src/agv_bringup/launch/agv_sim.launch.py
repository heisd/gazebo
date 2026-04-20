#!/usr/bin/env python3

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description():
    desc_dir = get_package_share_directory('agv_description')
    urdf = os.path.join(desc_dir, 'urdf', 'agv_robot.urdf.xacro')
    result = subprocess.run(['xacro', urdf], capture_output=True, text=True)
    return result.stdout


def generate_launch_description():
    gaz_dir = get_package_share_directory('agv_gazebo')
    world = os.path.join(gaz_dir, 'worlds', 'warehouse.world')

    robot_desc = get_robot_description()
    use_sim = LaunchConfiguration('use_sim_time', default='true')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim}],
        output='screen')

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim}],
        output='screen')

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'agv_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.12'],
        output='screen')

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen')

    load_ddc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        LogInfo(msg='[AGV] 启动 Gazebo 仿真底座...'),
        gazebo,
        TimerAction(period=5.0, actions=[rsp]),
        TimerAction(period=5.5, actions=[jsp]),
        TimerAction(period=7.0, actions=[spawn]),
        TimerAction(period=12.0, actions=[load_jsb]),
        TimerAction(period=14.0, actions=[load_ddc]),
        TimerAction(period=16.0, actions=[rviz]),
    ])
