"""ZED 2i + RoboSense M1 + camera perception and mapping, without vehicle control."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from detection_fusion.calibration import load_calibration


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    model = Path(value('model_path')).expanduser()
    calibration = Path(value('calibration_path')).expanduser()
    if not model.is_file() or model.suffix.lower() not in ('.pt', '.onnx', '.engine'):
        raise ValueError('model_path must point to an existing PT, ONNX, or TensorRT engine file')
    detector_backend = value('detector_backend')
    if detector_backend == 'auto':
        detector_backend = ('cpp' if model.suffix.lower() == '.engine' and
                            model.stem.lower().startswith('lwdetr') else 'python')
    if detector_backend == 'cpp' and (model.suffix.lower() != '.engine' or
                                      not model.stem.lower().startswith('lwdetr')):
        raise ValueError('C++ detector requires a LW-DETR TensorRT engine')
    if value('debug_orange') == 'true' and value('fusion_backend') != 'cpp':
        raise ValueError('debug_orange requires fusion_backend:=cpp')
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
        Node(package='camera_detection', executable=(
            'lwdetr_tensorrt_node' if detector_backend == 'cpp' else 'yolov8_node'),
             parameters=[{'model_path': str(model), 'red_color': red_color,
                'image_topic': value('image_topic'), 'confidence_threshold': confidence,
                'inference_threads': threads,
                'model_input_width': int(value('model_input_width')),
                'model_input_height': int(value('model_input_height')),
                'device': value('device'), 'gpu_device_id': int(value('gpu_device_id')),
                'publish_annotated_image': value('publish_annotated_image') == 'true'}], output='screen'),
        Node(package='camera_detection', executable=(
            'stereo_detection_adapter_cpp' if value('adapter_backend') == 'cpp'
            else 'stereo_detection_adapter'), parameters=[{
             'depth_topic': value('depth_topic'), 'info_topic': value('info_topic')}], output='screen'),
        Node(package='lidar_detection', executable='lidar_detection_node', parameters=[
             str(Path(get_package_share_directory('lidar_detection')) / 'config/lidar_detection.yaml'),
             {'input_topic': value('lidar_topic'), 'output_topic': '/perception/lidar/cones_raw',
              'use_ransac': True,
              'ransac_distance_threshold': float(value('lidar_ransac_distance_threshold')),
              'ground_max_tilt_deg': float(value('lidar_ground_max_tilt_deg')),
              'voxel_leaf_size': float(value('lidar_voxel_leaf_size')),
              'voxel_before_ground': value('lidar_voxel_before_ground') == 'true',
              'ransac_max_iterations': int(value('lidar_ransac_max_iterations')),
              'ransac_probability': float(value('lidar_ransac_probability')),
              'cluster_tolerance': float(value('lidar_cluster_tolerance')),
              'min_cluster_size': int(value('lidar_min_cluster_size')),
              'max_cluster_size': int(value('lidar_max_cluster_size')),
              'max_cone_width': float(value('lidar_max_cone_width')),
              'min_cone_height': float(value('lidar_min_cone_height')),
              'max_cone_height': float(value('lidar_max_cone_height')),
              'max_detection_range': float(value('lidar_max_detection_range')),
              'profile_stages': value('profile_lidar') == 'true'}], output='screen'),
        Node(package='detection_fusion', executable=(
             'detection_fusion_node_cpp' if value('fusion_backend') == 'cpp'
             else 'detection_fusion_node'), parameters=[
             str(share / 'config/fusion.yaml'), {
                'fuse_positions': value('fusion_fuse_positions') == 'true',
                'max_wait_sec': wait,
                'sync_slop_sec': float(value('fusion_sync_slop_sec')),
                'max_match_distance': float(value('fusion_max_match_distance')),
                'mahalanobis_gate': float(value('fusion_mahalanobis_gate')),
                'pixel_margin': float(value('fusion_pixel_margin')),
                'ambiguity_margin': float(value('fusion_ambiguity_margin')),
                'min_detection_confidence': confidence,
                'min_color_probability': float(value('fusion_min_color_probability')),
                'publish_unmatched_lidar': value('publish_unmatched_lidar') == 'true',
                'pointcloud_topic': value('lidar_topic'), 'guided_clustering': True,
                'guided_voxel_size': float(value('guided_voxel_size')),
                'guided_cluster_tolerance': float(value('guided_cluster_tolerance')),
                'guided_depth_tolerance': float(value('guided_depth_tolerance')),
                'guided_min_cluster_size': int(value('guided_min_cluster_size')),
                'guided_max_cluster_size': int(value('guided_max_cluster_size')),
                'guided_max_width': float(value('guided_max_width')),
                'guided_min_height': float(value('guided_min_height')),
                'guided_max_height': float(value('guided_max_height')),
                'debug_orange': value('debug_orange') == 'true',
                'debug_sync_slop_sec': float(value('debug_sync_slop_sec')),
                'debug_pixel_margin': float(value('debug_pixel_margin'))}],
             output='screen'),
        Node(package='cone_map_builder', executable='cone_map_builder_node', name='cone_map_builder',
             parameters=[str(Path(get_package_share_directory('cone_map_builder')) / 'config/cone_map_builder.yaml'),
                {'assign_colors': False, 'allow_semantic_color_correction': True,
                 'semantic_color_confirmation_hits': 3}],
             remappings=[('/perception/lidar/cones', '/perception/fused/cones'),
                         ('/localization/pose', value('localization_pose_topic'))],
             condition=UnlessCondition(LaunchConfiguration('debug_orange')), output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', value('rviz_config')],
             condition=IfCondition(LaunchConfiguration('launch_rviz')), output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path', description='Absolute PT, ONNX, or TensorRT engine path'),
        DeclareLaunchArgument('detector_backend', default_value='auto',
            choices=['auto', 'cpp', 'python'],
            description='auto uses C++ for LW-DETR engines; python keeps the original node'),
        DeclareLaunchArgument('adapter_backend', default_value='cpp',
            choices=['cpp', 'python'],
            description='C++ registered-depth adapter or original Python adapter'),
        DeclareLaunchArgument('fusion_backend', default_value='cpp',
            choices=['cpp', 'python'],
            description='C++ late fusion or original Python fusion node'),
        DeclareLaunchArgument('calibration_path', description='camera-from-lidar YAML path'),
        DeclareLaunchArgument('red_color', default_value='3', description='0 UNKNOWN, 3 ORANGE'),
        DeclareLaunchArgument('start_drivers', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('launch_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=str(Path(
            get_package_share_directory('detection_fusion')) / 'config/hardware.rviz')),
        DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/rgb/image_rect_color'),
        DeclareLaunchArgument('lidar_topic', default_value='/rslidar_points'),
        DeclareLaunchArgument('lidar_voxel_before_ground', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('lidar_voxel_leaf_size', default_value='0.08'),
        DeclareLaunchArgument('lidar_ransac_distance_threshold', default_value='0.08'),
        DeclareLaunchArgument('lidar_ground_max_tilt_deg', default_value='5.0'),
        DeclareLaunchArgument('lidar_ransac_max_iterations', default_value='500'),
        DeclareLaunchArgument('lidar_ransac_probability', default_value='0.999'),
        DeclareLaunchArgument('lidar_cluster_tolerance', default_value='0.4'),
        DeclareLaunchArgument('lidar_min_cluster_size', default_value='3'),
        DeclareLaunchArgument('lidar_max_cluster_size', default_value='200'),
        DeclareLaunchArgument('lidar_max_cone_width', default_value='0.5'),
        DeclareLaunchArgument('lidar_min_cone_height', default_value='0.1'),
        DeclareLaunchArgument('lidar_max_cone_height', default_value='0.6'),
        DeclareLaunchArgument('lidar_max_detection_range', default_value='20.0'),
        DeclareLaunchArgument('profile_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('debug_orange', default_value='false', choices=['true', 'false'],
            description='Publish low-latency orange cone position from raw cloud and camera box'),
        DeclareLaunchArgument('depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('info_topic', default_value='/zed/zed_node/rgb/camera_info'),
        DeclareLaunchArgument('localization_pose_topic', default_value='/zed/zed_node/pose',
            description='Map-frame PoseStamped used by the builder; ZED tracking for this standalone rig'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('model_input_width', default_value='0',
            description='PT/engine input width before stride padding; zero keeps the model default'),
        DeclareLaunchArgument('model_input_height', default_value='0',
            description='PT/engine input height before stride padding; zero keeps the model default'),
        DeclareLaunchArgument('publish_unmatched_lidar', default_value='false', choices=['true', 'false'],
            description='Publish LiDAR clusters without a matched camera box'),
        DeclareLaunchArgument('inference_threads', default_value='4'),
        DeclareLaunchArgument('device', default_value='cuda', choices=['cuda', 'cpu']),
        DeclareLaunchArgument('gpu_device_id', default_value='0'),
        DeclareLaunchArgument('fusion_wait_sec', default_value='0.10',
            description='Maximum wait for a synchronized camera result; timestamp slop is 30 ms'),
        DeclareLaunchArgument('fusion_sync_slop_sec', default_value='0.03'),
        DeclareLaunchArgument('fusion_max_match_distance', default_value='0.8'),
        DeclareLaunchArgument('fusion_mahalanobis_gate', default_value='11.345'),
        DeclareLaunchArgument('fusion_pixel_margin', default_value='8.0'),
        DeclareLaunchArgument('fusion_ambiguity_margin', default_value='0.15'),
        DeclareLaunchArgument('fusion_min_color_probability', default_value='0.6'),
        DeclareLaunchArgument('fusion_fuse_positions', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('guided_voxel_size', default_value='0.05'),
        DeclareLaunchArgument('guided_cluster_tolerance', default_value='0.15'),
        DeclareLaunchArgument('guided_depth_tolerance', default_value='0.4'),
        DeclareLaunchArgument('guided_min_cluster_size', default_value='5'),
        DeclareLaunchArgument('guided_max_cluster_size', default_value='200'),
        DeclareLaunchArgument('guided_max_width', default_value='0.5'),
        DeclareLaunchArgument('guided_min_height', default_value='0.08'),
        DeclareLaunchArgument('guided_max_height', default_value='0.7'),
        DeclareLaunchArgument('debug_sync_slop_sec', default_value='0.06'),
        DeclareLaunchArgument('debug_pixel_margin', default_value='8.0'),
        DeclareLaunchArgument('publish_annotated_image', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
