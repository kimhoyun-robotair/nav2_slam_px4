#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock if running in Gazebo/IGN'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    rtabmap_pkg_share = FindPackageShare('rtabmap_nav2_px4')
    px4_off_pkg_share = FindPackageShare('px4_offboard')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_pkg_share, 'launch', 'spawn_robot.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_pkg_share, 'launch', 'rtabmap.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_pkg_share, 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,       # nav2 측 인자는 다를 수 있음
        }.items()
    )

    offboard_vel_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([px4_off_pkg_share,
                                  'offboard_velocity_control.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )


    # ─────────────────────────────────────────────
    # 4) LaunchDescription 반환
    # ─────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time_arg,
        spawn_robot,
        rtabmap,
        navigation,
        offboard_vel_ctrl,
    ])
