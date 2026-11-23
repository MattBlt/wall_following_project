import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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

        # con ekf:=false in simulazione non esiste la TF odom -> base_link
        # (il plugin non la pubblica piu'): RViz con Fixed Frame odom resta vuoto
        DeclareLaunchArgument('ekf', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'ekf.launch.py')),
            launch_arguments={'topic_map': 'topics_sim.yaml',
                              'use_sim_time': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('ekf'))),

        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'rviz.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))),

    ])
