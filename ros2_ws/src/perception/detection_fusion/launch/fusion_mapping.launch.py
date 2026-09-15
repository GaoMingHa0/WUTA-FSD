"""Hardware-neutral mapping: drivers, YOLOv8 and localization TF supplied externally."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def config(package, filename):
    return PathJoinSubstitution([FindPackageShare(package), 'config', filename])


def generate_launch_description():
    return LaunchDescription([
        Node(package='lidar_detection', executable='lidar_detection_node',
             parameters=[config('lidar_detection', 'lidar_detection.yaml'),
                         {'output_topic': '/perception/lidar/cones_raw'}]),
        Node(package='camera_detection', executable='stereo_detection_adapter'),
        Node(package='detection_fusion', executable='detection_fusion_node',
             parameters=[config('detection_fusion', 'fusion.yaml'),
                         {'fuse_positions': False}], output='screen'),
        Node(package='cone_map_builder', executable='cone_map_builder_node', name='cone_map_builder',
             parameters=[config('cone_map_builder', 'cone_map_builder.yaml'),
                         {'assign_colors': False, 'allow_semantic_color_correction': True,
                          'semantic_color_confirmation_hits': 3}],
             remappings=[('/perception/lidar/cones', '/perception/fused/cones')], output='screen'),
    ])
