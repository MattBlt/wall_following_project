import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg = get_package_share_directory('limo_wf_sim')

    xacro_file = os.path.join(pkg, 'urdf', 'limo_diff', 'limo_diff.xacro')
    default_world = os.path.join(pkg, 'worlds', 'closed_room.world')

    # Posa di spawn di base_link nel frame del mondo. Default: angolo della stanza
    # all'origine in closed_room.world, 0.225 m da wall9 (x=0) e 0.375 m da wall1 (y=0),
    # muso verso +x. L'interno sta sotto y=0, da cui la y negativa.
    args = [
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('x', default_value='0.30'),
        DeclareLaunchArgument('y', default_value='-0.45'),
        DeclareLaunchArgument('z', default_value='0.20'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('noise_seed', default_value='0'),
    ]

    sim_time = {'use_sim_time': True}

    return LaunchDescription(args + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=UnlessCondition(LaunchConfiguration('headless'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': Command(['xacro ', xacro_file]), **sim_time}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[sim_time]),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'limo_gazebo', '-topic', 'robot_description',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')]),
        Node(package='limo_wf_sim', executable='twist_watchdog.py', name='twist_watchdog'),
        TimerAction(period=8.0, condition=UnlessCondition(LaunchConfiguration('headless')), actions=[
            ExecuteProcess(cmd=['gz', 'camera', '-c', 'user_camera', '-f', 'limo_gazebo'])]),
        Node(package='limo_wf_sim', executable='odom_noise.py', name='odom_noise',
             parameters=[{**sim_time,
                          'noise_seed': ParameterValue(LaunchConfiguration('noise_seed'),
                                                 value_type=int)}]),
    ])