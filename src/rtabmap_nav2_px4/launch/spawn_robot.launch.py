import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_rtabmap_nav2_px4 = get_package_share_directory('rtabmap_nav2_px4')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_rtabmap_nav2_px4)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

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
        pkg_rtabmap_nav2_px4,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
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
        package="rtabmap_nav2_px4",
        executable="odom_baselink_publisher.py",
        name="map_baselink_publisher",
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    image_transformer_node = Node(
        package='rtabmap_nav2_px4',
        executable='image_transform.py',
        name='image_transform',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    velocity_transformer_node = Node(
        package='rtabmap_nav2_px4',
        executable='velocity_transform.py',
        name='velocity_transform',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    tracking_log_node = Node(
        package='rtabmap_nav2_px4',
        executable='tracking_log.py',
        name='px4_topic_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        prefix=['gnome-terminal -- ']
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher)
    launchDescriptionObject.add_action(tf_broadcaster_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(relay_camera_info_node)
    launchDescriptionObject.add_action(image_transformer_node)
    launchDescriptionObject.add_action(velocity_transformer_node)
    launchDescriptionObject.add_action(tracking_log_node)
    
    return launchDescriptionObject