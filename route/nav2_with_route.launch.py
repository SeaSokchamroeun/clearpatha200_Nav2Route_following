#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

NAMESPACE = 'a200_1103'
HOME      = os.path.expanduser('~')
PARAMS    = os.path.join(HOME, 'clearpath', 'route', 'route_server_params.yaml')
GRAPH     = os.path.join(HOME, 'clearpath', 'route', 'fullLoopRoute1.geojson')

def generate_launch_description():

    # Include the existing clearpath nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('clearpath_nav2_demos'),
            '/launch/nav2.launch.py'
        ])
    )

    # Route server registered under same namespace
    route_server = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        namespace=NAMESPACE,
        output='screen',
        parameters=[
            PARAMS,
            {'graph_filepath': GRAPH,
             'use_sim_time': True}   # ← critical for simulation clock
        ],
        remappings=[
            ('/tf',        f'/{NAMESPACE}/tf'),
            ('/tf_static', f'/{NAMESPACE}/tf_static'),
        ]
    )

    return LaunchDescription([nav2_launch, route_server])
