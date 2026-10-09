import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('limo_wf_bringup')

    # .perform(context) risolve l'argomento in una sola stringa vera
    topic_map = LaunchConfiguration('topic_map').perform(context)
    use_sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    with open(os.path.join(pkg, 'config', topic_map)) as f:
        cfg = yaml.safe_load(f)
    t, fr = cfg['topics'], cfg['frames']

    common = {'use_sim_time': use_sim}

    return [
        Node(package='limo_wf', executable='wall_detection', name='wall_detection',
             output='screen',
             parameters=[common, {'laser_frame': fr['laser']}],
             remappings=[('scan', t['scan']),
                         ('wall_info', '/wall_info')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('topic_map', default_value='topics_sim.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
