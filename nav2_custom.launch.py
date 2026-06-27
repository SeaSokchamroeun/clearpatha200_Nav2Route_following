#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from ament_index_python.packages import get_package_share_directory

NAMESPACE = 'a200_1103'
HOME      = os.path.expanduser('~')
PARAMS    = os.path.join(HOME, 'clearpath', 'nav2_custom.yaml')

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
]

def launch_setup(context, *args, **kwargs):
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2 = GroupAction([
        PushRosNamespace(NAMESPACE),
        SetRemap(f'/{NAMESPACE}/odom', f'/{NAMESPACE}/platform/odom'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments=[
                ('use_sim_time',    LaunchConfiguration('use_sim_time')),
                ('params_file',     PARAMS),
                ('use_composition', 'False'),
                ('namespace',       NAMESPACE),
            ]
        ),
    ])
    return [nav2]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
