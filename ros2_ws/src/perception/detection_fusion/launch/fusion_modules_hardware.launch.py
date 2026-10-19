"""实车模块配置入口：算法参数只从各包config读取，显式部署选项除外。"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from detection_fusion.calibration import load_calibration


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    root = Path(arg('config_root')).expanduser()
    paths = {
        'camera': root / 'perception/camera_detection/config/camera_detection.yaml',
        'adapter': root / 'perception/camera_detection/config/stereo_adapter.yaml',
        'lidar': root / 'perception/lidar_detection/config/lidar_detection.yaml',
        'fusion': root / 'perception/detection_fusion/config/fusion.yaml',
        'zed': root / 'perception/detection_fusion/config/zed_hardware.yaml',
        'rslidar': root / 'perception/detection_fusion/config/rsm1_hardware.yaml',
    }
    values = {}
    for key in ('camera', 'adapter', 'lidar', 'fusion'):
        document = yaml.safe_load(paths[key].read_text())
        values[key] = next(v['ros__parameters'] for v in document.values()
                           if isinstance(v, dict) and 'ros__parameters' in v)
    supplied_model = arg('model_path')
    model = Path(supplied_model or values['camera']['model_path']).expanduser()
    if not model.is_absolute():
        model = (paths['camera'].parent / model).resolve()
    if not model.is_file() or model.suffix.lower() not in ('.pt', '.onnx', '.engine'):
        raise ValueError('相机模块model_path必须指向存在的PT/ONNX/engine文件')
    backend = arg('detector_backend')
    if backend == 'auto':
        backend = 'cpp' if model.suffix.lower() == '.engine' and model.stem.lower().startswith('lwdetr') else 'python'
    if backend == 'cpp' and (model.suffix.lower() != '.engine' or not model.stem.lower().startswith('lwdetr')):
        raise ValueError('C++检测后端仅支持LW-DETR引擎')
    fusion = values['fusion']
    if fusion.get('debug_red') and fusion.get('debug_orange'):
        raise ValueError('两种锥桶调试模式不能同时开启')
    if arg('fusion_backend') != 'cpp' and (fusion.get('debug_red') or fusion.get('debug_orange')):
        raise ValueError('锥桶调试需要C++融合后端')
    camera_frame, lidar_frame, translation, quaternion = load_calibration(Path(arg('calibration_path')))
    driver = yaml.safe_load(paths['rslidar'].read_text())['lidar'][0]['ros']
    if driver['ros_frame_id'] != lidar_frame:
        raise ValueError('RSM1驱动坐标帧与相机雷达外参不一致')
    tf_args = []
    for key, number in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), [*translation, *quaternion]):
        tf_args += ['--' + key, str(number)]
    tf_args += ['--frame-id', camera_frame, '--child-frame-id', lidar_frame]
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
            get_package_share_directory('zed_wrapper')) / 'launch/zed_camera.launch.py')),
            launch_arguments={'camera_model': 'zed2i', 'camera_name': 'zed',
                'ros_params_override_path': str(paths['zed']),
                'publish_tf': arg('zed_publish_tf'), 'publish_map_tf': arg('zed_publish_map_tf')}.items(),
            condition=IfCondition(LaunchConfiguration('start_drivers'))),
        Node(package='rslidar_sdk', executable='rslidar_sdk_node',
             parameters=[{'config_path': str(paths['rslidar'])}],
             condition=IfCondition(LaunchConfiguration('start_drivers')), output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_lidar_extrinsics', arguments=tf_args, output='screen',
             condition=IfCondition(LaunchConfiguration('start_drivers'))),
        Node(package='camera_detection', executable='lwdetr_tensorrt_node' if backend == 'cpp' else 'yolov8_node',
             parameters=[str(paths['camera']), {'model_path': str(model)}], output='screen',
             condition=IfCondition(LaunchConfiguration('start_algorithms'))),
        Node(package='camera_detection', executable='stereo_detection_adapter_cpp' if arg('adapter_backend') == 'cpp'
             else 'stereo_detection_adapter', parameters=[str(paths['adapter'])], output='screen',
             condition=IfCondition(LaunchConfiguration('start_algorithms'))),
        Node(package='lidar_detection', executable='lidar_detection_node',
             parameters=[str(paths['lidar'])], output='screen',
             condition=IfCondition(LaunchConfiguration('start_algorithms'))),
        Node(package='detection_fusion', executable='detection_fusion_node_cpp' if arg('fusion_backend') == 'cpp'
             else 'detection_fusion_node', parameters=[str(paths['fusion'])], output='screen',
             condition=IfCondition(LaunchConfiguration('start_algorithms'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_root', description='FSD ros2_ws/src目录，直接读取源码模块config'),
        DeclareLaunchArgument('model_path', default_value='', description='仅显式选择其他模型时覆盖相机模块配置'),
        DeclareLaunchArgument('calibration_path', description='相机雷达物理外参文件'),
        DeclareLaunchArgument('detector_backend', default_value='auto', choices=['auto', 'cpp', 'python']),
        DeclareLaunchArgument('adapter_backend', default_value='cpp', choices=['cpp', 'python']),
        DeclareLaunchArgument('fusion_backend', default_value='cpp', choices=['cpp', 'python']),
        DeclareLaunchArgument('start_drivers', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_algorithms', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('zed_publish_tf', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('zed_publish_map_tf', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
