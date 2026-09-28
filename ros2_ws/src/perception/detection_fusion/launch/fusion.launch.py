"""Hardware-neutral entry: supply raw LiDAR detections, YOLO adapter and TF externally."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([Node(
        package='detection_fusion', executable='detection_fusion_node',
        parameters=[PathJoinSubstitution([FindPackageShare('detection_fusion'), 'config', 'fusion.yaml'])],
        output='screen')])
