import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim = get_package_share_directory('limo_wf_sim')
    bringup = get_package_share_directory('limo_wf_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim, 'launch', 'gazebo.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'wall_following.launch.py')),
            launch_arguments={'topic_map': 'topics_sim.yaml',
                              'use_sim_time': 'true'}.items()),

    ])
