from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/hesai/pandar',
        description='Raw point cloud topic from Hesai 128-line LiDAR'
    )

    kiss_icp_config = PathJoinSubstitution([
        FindPackageShare('kiss_icp_wrapper'), 'config', 'kiss_icp_hesai128.yaml'
    ])

    ekf_config = PathJoinSubstitution([
        FindPackageShare('localization_manager'), 'config', 'ekf.yaml'
    ])

    # KISS-ICP node (from kiss-icp package)
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        parameters=[kiss_icp_config],
        remappings=[
            ('pointcloud_topic', LaunchConfiguration('pointcloud_topic')),
        ],
        output='screen',
    )

    # robot_localization EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[ekf_config],
        output='screen',
    )

    # Localization manager (mode switcher)
    localization_manager_node = Node(
        package='localization_manager',
        executable='localization_manager_node',
        name='localization_manager',
        output='screen',
    )

    return LaunchDescription([
        pointcloud_topic_arg,
        kiss_icp_node,
        ekf_node,
        localization_manager_node,
    ])
