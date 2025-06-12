import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


def generate_launch_description() -> LaunchDescription:
    # custom package for autonomous exploration drone simulation
    pkg_nav2_slam_px4 = get_package_share_directory('rtabmap_nav2_px4')
    nav2_slam_px4_launch_dir = os.path.join(pkg_nav2_slam_px4, 'launch')
    # Get the keyboard teleop package for PX4
    px4_keyboard_teleopdir = get_package_share_directory('px4_offboard')
    # Get the launch directory+
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Remappings for tf
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Combine all parameter rewrites
    param_rewrites = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav2_slam_px4, 'config', 'navigation.yaml'),
        description='Path to the ROS2 parameters file for all nodes',
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack',
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup',
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn a node on crash',
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level')
    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value='', description='Path to the graph file to load'
    )

    # Define bringup actions
    bringup_cmd_group = GroupAction([
        # Container for composable nodes
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen',
        ),
        # SLAM launch unconditionally
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_slam_px4_launch_dir, 'spawn_robot.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_slam_px4_launch_dir, 'rtabmap.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(px4_keyboard_teleopdir, 
                             'offboard_velocity_control.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'graph': LaunchConfiguration('graph'),
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container',
            }.items(),
        ),
    ])

    # Assemble LaunchDescription
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    for cmd in [
        declare_use_sim_time_cmd,
        declare_graph_file_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
    ]:
        ld.add_action(cmd)
    ld.add_action(bringup_cmd_group)

    return ld