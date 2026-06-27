#!/usr/bin/env python3
# ============================================================
#  route_server.launch.py — Husky A200 (a200_1103)
#
#  Run AFTER nav2.launch.py is already up.
#
#  Default graph (testLoop1):
#    ros2 launch ~/clearpath/route/route_server.launch.py
#
#  Override graph from CLI:
#    ros2 launch ~/clearpath/route/route_server.launch.py \
#      graph:=$HOME/clearpath/route/testStraight.geojson
# ============================================================

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE  = 'a200_1103'
HOME       = os.path.expanduser('~')
ROUTE_DIR  = os.path.join(HOME, 'clearpath', 'route')
PARAMS     = os.path.join(ROUTE_DIR, 'route_server_params.yaml')


def generate_launch_description():

    graph_arg = DeclareLaunchArgument(
        'graph',
        default_value=os.path.join(ROUTE_DIR, 'testLoop1.geojson'),
        description='Full path to GeoJSON route graph file'
    )

    route_server = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        namespace=NAMESPACE,
        output='screen',
        parameters=[
            PARAMS,
            {'graph_filepath': LaunchConfiguration('graph')}
        ],
        remappings=[
            ('/tf',        f'/{NAMESPACE}/tf'),
            ('/tf_static', f'/{NAMESPACE}/tf_static'),
            ('odom',       f'/{NAMESPACE}/platform/odom'),
            ('cmd_vel',    f'/{NAMESPACE}/cmd_vel'),
        ]
    )

    return LaunchDescription([graph_arg, route_server])