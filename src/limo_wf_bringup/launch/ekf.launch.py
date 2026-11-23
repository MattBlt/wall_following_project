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
    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    with open(os.path.join(pkg, 'config', topic_map)) as f:
        cfg = yaml.safe_load(f)
    topics, frames = cfg['topics'], cfg['frames']

    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')

    # l'ordine conta: il dizionario viene dopo il YAML, quindi topic e frame
    # presi dal file dei topic vincono su quelli scritti in ekf.yaml
    return [
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen',
             parameters=[ekf_yaml, {
                 'use_sim_time': use_sim,
                 'odom0': topics['ekf_odom'],
                 'imu0': topics['ekf_imu'],
                 'map_frame': frames['map'],
                 'odom_frame': frames['odom'],
                 'base_link_frame': frames['base'],
                 'world_frame': frames['odom']}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('topic_map', default_value='topics_sim.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
