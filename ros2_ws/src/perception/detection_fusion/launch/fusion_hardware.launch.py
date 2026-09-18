"""ZED 2i + RoboSense M1 + ONNX perception and mapping, without vehicle control."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from detection_fusion.calibration import load_calibration


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    model = Path(value('model_path')).expanduser()
    calibration = Path(value('calibration_path')).expanduser()
    if not model.is_file() or model.suffix.lower() not in ('.pt', '.onnx'):
        raise ValueError('model_path must point to an existing PT or ONNX file')
    camera_frame, lidar_frame, translation, quaternion = load_calibration(calibration)
    if lidar_frame != 'rslidar':
        raise ValueError('The supplied M1 config publishes rslidar; update it before using another frame')
    share = Path(get_package_share_directory('detection_fusion'))
    red_color = int(value('red_color'))
    if red_color not in (0, 3):
        raise ValueError('red_color must be 0 (UNKNOWN) or 3 (ORANGE)')
    confidence = float(value('confidence_threshold'))
    threads = int(value('inference_threads'))
    wait = float(value('fusion_wait_sec'))
    if not 0 < confidence < 1 or threads < 1 or not 0 < wait <= 1.5:
        raise ValueError('Require 0 < confidence_threshold < 1, inference_threads >= 1, 0 < fusion_wait_sec <= 1.5')
    for name in ('image_topic', 'lidar_topic', 'depth_topic', 'info_topic', 'localization_pose_topic'):
        if not value(name).startswith('/'):
            raise ValueError(name + ' must be an absolute ROS topic')
    tf_arguments = []
    for key, number in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), [*translation, *quaternion]):
        tf_arguments += ['--' + key, str(number)]
    tf_arguments += ['--frame-id', camera_frame, '--child-frame-id', lidar_frame]
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
            get_package_share_directory('zed_wrapper')) / 'launch/zed_camera.launch.py')),
            launch_arguments={'camera_model': 'zed2i', 'camera_name': 'zed',
                'ros_params_override_path': str(share / 'config/zed_hardware.yaml'),
                'publish_tf': 'true', 'publish_map_tf': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('start_drivers'))),
        Node(package='rslidar_sdk', executable='rslidar_sdk_node',
             parameters=[{'config_path': str(share / 'config/rsm1_hardware.yaml')}],
             condition=IfCondition(LaunchConfiguration('start_drivers')), output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_lidar_extrinsics', arguments=tf_arguments, output='screen'),
        Node(package='camera_detection', executable='yolov8_node',
             parameters=[{'model_path': str(model), 'red_color': red_color,
                'image_topic': value('image_topic'), 'confidence_threshold': confidence,
                'inference_threads': threads,
                'device': value('device'), 'gpu_device_id': int(value('gpu_device_id')),
                'publish_annotated_image': value('publish_annotated_image') == 'true'}], output='screen'),
        Node(package='camera_detection', executable='stereo_detection_adapter', parameters=[{
             'depth_topic': value('depth_topic'), 'info_topic': value('info_topic')}], output='screen'),
        Node(package='lidar_detection', executable='lidar_detection_node', parameters=[
             str(Path(get_package_share_directory('lidar_detection')) / 'config/lidar_detection.yaml'),
             {'input_topic': value('lidar_topic'), 'output_topic': '/perception/lidar/cones_raw',
              'use_ransac': True, 'ransac_distance_threshold': 0.08,
              'ground_max_tilt_deg': 5.0, 'voxel_leaf_size': 0.05}], output='screen'),
        Node(package='detection_fusion', executable='detection_fusion_node', parameters=[
             str(share / 'config/fusion.yaml'), {'fuse_positions': False, 'max_wait_sec': wait,
                'min_detection_confidence': confidence,
                'min_color_probability': 0.6,
                'publish_unmatched_lidar': value('publish_unmatched_lidar') == 'true',
                'pointcloud_topic': value('lidar_topic'), 'guided_clustering': True}],
             output='screen'),
        Node(package='cone_map_builder', executable='cone_map_builder_node', name='cone_map_builder',
             parameters=[str(Path(get_package_share_directory('cone_map_builder')) / 'config/cone_map_builder.yaml'),
                {'assign_colors': False, 'allow_semantic_color_correction': True,
                 'semantic_color_confirmation_hits': 3}],
             remappings=[('/perception/lidar/cones', '/perception/fused/cones'),
                         ('/localization/pose', value('localization_pose_topic'))], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', value('rviz_config')],
             condition=IfCondition(LaunchConfiguration('launch_rviz')), output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', description='Absolute PT or ONNX weights path'),
        DeclareLaunchArgument('calibration_path', description='camera-from-lidar YAML path'),
        DeclareLaunchArgument('red_color', default_value='3', description='0 UNKNOWN, 3 ORANGE'),
        DeclareLaunchArgument('start_drivers', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('launch_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=str(Path(
            get_package_share_directory('detection_fusion')) / 'config/hardware.rviz')),
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/rgb/image_rect_color'),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('info_topic', default_value='/zed/zed_node/rgb/camera_info'),
        DeclareLaunchArgument('localization_pose_topic', default_value='/zed/zed_node/pose',
            description='Map-frame PoseStamped used by the builder; ZED tracking for this standalone rig'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('publish_unmatched_lidar', default_value='false', choices=['true', 'false'],
            description='Publish LiDAR clusters without a matched camera box'),
        DeclareLaunchArgument('inference_threads', default_value='4'),
        DeclareLaunchArgument('device', default_value='cuda', choices=['cuda', 'cpu']),
        DeclareLaunchArgument('gpu_device_id', default_value='0'),
        DeclareLaunchArgument('fusion_wait_sec', default_value='1.2',
            description='Wait for YOLO results; matching exposure slop remains 60 ms'),
        DeclareLaunchArgument('publish_annotated_image', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
