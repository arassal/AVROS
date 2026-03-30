from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    default_repo_root = '/home/alexander/github/AVROS/src/avros_perception/avros_perception'

    return LaunchDescription([
        DeclareLaunchArgument(
            'venv_python',
            default_value='/home/alexander/github/av-perception/.venv/bin/python',
            description='Python interpreter for the segmentation runtime',
        ),
        DeclareLaunchArgument(
            'project_root',
            default_value='/home/alexander/Desktop/seg',
            description='Path to the segmentation project root with utils and assets',
        ),
        DeclareLaunchArgument(
            'image_dir',
            default_value='/home/alexander/Desktop/seg/img',
            description='Image folder used for the demo segmentation stream',
        ),
        DeclareLaunchArgument(
            'weights_path',
            default_value='/home/alexander/Desktop/seg/data/weights/yolopv2.pt',
            description='TorchScript YOLOPv2 weights path',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cpu',
            description='Inference device',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='1.0',
            description='Segmentation publish rate',
        ),
        DeclareLaunchArgument(
            'node_source_dir',
            default_value=default_repo_root,
            description='Source directory containing the AVROS perception Python nodes',
        ),

        ExecuteProcess(
            cmd=[
                LaunchConfiguration('venv_python'),
                PathJoinSubstitution([LaunchConfiguration('node_source_dir'), 'seg_demo_node.py']),
                '--ros-args',
                '-p', ['project_root:=', LaunchConfiguration('project_root')],
                '-p', ['image_dir:=', LaunchConfiguration('image_dir')],
                '-p', ['weights_path:=', LaunchConfiguration('weights_path')],
                '-p', ['device:=', LaunchConfiguration('device')],
                '-p', ['publish_rate_hz:=', LaunchConfiguration('publish_rate_hz')],
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                LaunchConfiguration('venv_python'),
                PathJoinSubstitution([LaunchConfiguration('node_source_dir'), 'detections_to_obstacles.py']),
            ],
            output='screen',
        ),
    ])
