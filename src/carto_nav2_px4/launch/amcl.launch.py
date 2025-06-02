import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    pkg_nav2_px4 = get_package_share_directory('nav2_px4')
    pkg_share = FindPackageShare(package='nav2_px4').find('nav2_px4')

    # Nav2 configuration
    namespace = LaunchConfiguration('namespace')
    use_respawn = LaunchConfiguration('use_respawn')
    lifecycle_nodes = ['map_server', 'amcl']
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    map_dir = os.path.join(pkg_share, 'map')
    map_file = 'turtlebot.yaml'
    param_dir = os.path.join(pkg_share, 'config')
    param_file = 'amcl.yaml'
    use_sim_time = LaunchConfiguration('use_sim_time')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value="",
        description="Top-level namespace"
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(map_dir, map_file),
        description='[localize] Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(param_dir, param_file),
        description='Full path to the ROS2 parameters file to use')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='nav2.rviz',
        description='RViz config file'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='x500_rtab.urdf',
        description='Name of the URDF description to load'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_nav2_px4,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_nav2_px4, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/x500_rtab_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/world/turtlebot3_world/model/x500_rtab_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/world/turtlebot3_world/model/x500_rtab_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/world/turtlebot3_world/model/x500_rtab_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/world/turtlebot3_world/model/x500_rtab_0/link/link/sensor/lidar_2d_v2/scan', '/scan'),
            ('/world/turtlebot3_world/model/x500_rtab_0/link/base_link/sensor/imu_sensor/imu', '/imu'),
            ('/world/turtlebot3_world/model/x500_rtab_0/link/base_link/sensor/navsat_sensor/navsat', '/fix'),
            ('/model/x500_rtab_0/odometry', '/odom'),
            ('/depth_camera/points', '/camera/depth_image/points'),
            ('/camera_info', '/camera/image/camera_info'),
        ]
    )

    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['/camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    tf_broadcaster_node = Node(
        package="nav2_px4",
        executable="odom_baselink_publisher.py",
        name="map_baselink_publisher",
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    image_transformer_node = Node(
        package='nav2_px4',
        executable='image_transform.py',
        name='image_transform',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    velocity_transformer_node = Node(
        package='nav2_px4',
        executable='velocity_transform.py',
        name='velocity_transform',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}],
    )


    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(declare_autostart_cmd)
    launchDescriptionObject.add_action(declare_map_yaml_cmd)
    launchDescriptionObject.add_action(declare_namespace_cmd)
    launchDescriptionObject.add_action(declare_params_file_cmd)
    launchDescriptionObject.add_action(declare_use_respawn_cmd)
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(map_server)
    launchDescriptionObject.add_action(amcl)
    launchDescriptionObject.add_action(lifecycle_manager)
    launchDescriptionObject.add_action(joint_state_publisher)
    launchDescriptionObject.add_action(tf_broadcaster_node)
    launchDescriptionObject.add_action(image_transformer_node)
    launchDescriptionObject.add_action(relay_camera_info_node)
    launchDescriptionObject.add_action(velocity_transformer_node)

    return launchDescriptionObject