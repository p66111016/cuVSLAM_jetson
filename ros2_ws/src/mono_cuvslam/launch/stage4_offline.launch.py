"""Replay a mono image sequence through PyCuVSLAM and show stage 4 outputs."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('mono_cuvslam'))
    rviz_config = str(share / 'rviz' / 'stage4.rviz')
    image_dir = LaunchConfiguration('image_dir')
    calibration_file = LaunchConfiguration('calibration_file')
    start_index = LaunchConfiguration('start_index')
    max_frames = LaunchConfiguration('max_frames')
    source_fps = LaunchConfiguration('source_fps')
    replay_fps = LaunchConfiguration('replay_fps')
    metrics_output = LaunchConfiguration('metrics_output')
    trajectory_output = LaunchConfiguration('trajectory_output')
    path_history_poses = LaunchConfiguration('path_history_poses')
    use_rviz = LaunchConfiguration('use_rviz')

    tracker = Node(
        package='mono_cuvslam',
        executable='mono_cuvslam_node',
        output='screen',
        parameters=[{'input_reliability': 'reliable', 'publish_tf': True,
                     'max_path_poses': ParameterValue(path_history_poses, value_type=int)}],
    )
    metrics = Node(
        package='mono_cuvslam',
        executable='stage4_metrics',
        output='screen',
        parameters=[{
            'expected_frames': ParameterValue(max_frames, value_type=int),
            'output_file': metrics_output,
            'trajectory_file': trajectory_output,
        }],
    )
    sequence = Node(
        package='mono_camera_driver',
        executable='image_sequence_node',
        output='screen',
        parameters=[{
            'image_dir': image_dir,
            'calibration_file': calibration_file,
            'glob': 'firefly_*.png',
            'start_index': ParameterValue(start_index, value_type=int),
            'max_frames': ParameterValue(max_frames, value_type=int),
            'fps': ParameterValue(source_fps, value_type=float),
            'replay_fps': ParameterValue(replay_fps, value_type=float),
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )
    return LaunchDescription([
        DeclareLaunchArgument('image_dir', default_value='/home/user/2026-06-22VTOL/firefly'),
        DeclareLaunchArgument('calibration_file', default_value='/home/user/mono-cuvslam-ros2/config/firefly_20260622_assumed.yaml'),
        DeclareLaunchArgument('start_index', default_value='1'),
        DeclareLaunchArgument('max_frames', default_value='300'),
        DeclareLaunchArgument('source_fps', default_value='30.0'),
        DeclareLaunchArgument('replay_fps', default_value='0.0'),
        DeclareLaunchArgument('metrics_output', default_value=''),
        DeclareLaunchArgument('trajectory_output', default_value=''),
        DeclareLaunchArgument('path_history_poses', default_value='2000'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        tracker,
        metrics,
        TimerAction(period=2.0, actions=[sequence]),
        rviz,
    ])
