#!/usr/bin/env python3
"""
AGV 仓储系统完整启动文件 (Humble 最终修正版)
默认使用 Gazebo diff-drive 插件，避免 ros2_control 与 Gazebo 底盘插件争用。
"""
import os, subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, DeclareLaunchArgument,
                            IncludeLaunchDescription,
                            TimerAction, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_robot_description():
    desc_dir = get_package_share_directory('agv_description')
    urdf = os.path.join(desc_dir, 'urdf', 'agv_robot.urdf.xacro')
    # 使用 subprocess 解析 xacro
    result = subprocess.run(
        ['xacro', urdf, 'enable_ros2_control:=false'],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout

def generate_launch_description():
    desc_dir = get_package_share_directory('agv_description')
    gaz_dir  = get_package_share_directory('agv_gazebo')
    nav_dir  = get_package_share_directory('agv_navigation')

    world    = os.path.join(gaz_dir,  'worlds', 'warehouse.world')
    nav_cfg  = os.path.join(nav_dir,  'config', 'nav2_params.yaml')
    nav_launch = os.path.join(nav_dir, 'launch', 'navigation.launch.py')

    robot_desc = get_robot_description()
    use_sim    = LaunchConfiguration('use_sim_time', default='true')

    # 1. Gazebo 启动 (核心仿真环境)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # 2. Robot State Publisher (发布静态坐标变换)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim}],
        output='screen')

    # 2.1 Joint State Publisher
    # 为 continuous 轮关节发布 joint_states，确保 RViz 能看到 base_link -> wheel_link。
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': use_sim}],
        output='screen')

    # 3. 在 Gazebo 中生成 AGV 实体
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'agv_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.12'],
        output='screen')

    # 4. SLAM 建图 (异步 SLAM)
    slam_params = [nav_cfg, {'use_sim_time': use_sim}] if os.path.exists(nav_cfg) else [{'use_sim_time': use_sim}]
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=slam_params,
        remappings=[('/scan', '/agv/scan')],
        output='screen')

    # 5. Nav2 导航栈
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={
            'use_sim_time': use_sim,
            'params_file': nav_cfg,
            'autostart': 'true',
        }.items())

    # 6. AGV 调度节点
    scheduler = Node(
        package='agv_scheduler',
        executable='scheduler_node',
        name='agv_scheduler',
        parameters=[{'use_sim_time': use_sim}],
        output='screen')

    # 7. RViz2 (可视化)
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        LogInfo(msg='[AGV] 启动 Gazebo 仿真及控制链路...'),
        
        gazebo,
        TimerAction(period=5.0,  actions=[rsp]),
        TimerAction(period=5.5,  actions=[jsp]),
        TimerAction(period=7.0,  actions=[spawn]),
        TimerAction(period=12.0, actions=[slam]),
        TimerAction(period=16.0, actions=[nav2]),
        TimerAction(period=20.0, actions=[scheduler]),
        TimerAction(period=22.0, actions=[rviz]),
    ])
