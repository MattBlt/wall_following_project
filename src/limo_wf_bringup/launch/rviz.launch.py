import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('limo_wf_bringup'),
                            'rviz', 'wall_following.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])