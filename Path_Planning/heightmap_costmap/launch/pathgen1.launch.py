#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('heightmap_costmap')
    
    # A* planner node
    planner_node = Node(
        package='heightmap_costmap',
        executable='a_star',
        name='optimal_planner',
        output='screen',
        parameters=[{
            'total_cost_csv': os.path.join(pkg_dir, 'total_cost.csv'),
            'resolution': 0.05,
            'origin_x': 0.0,
            'origin_y': 0.0,
            'start_x': 442,
            'start_y': 461,
            'via1_x': 371,
            'via1_y': 184,
            'via2_x': 205,
            'via2_y': 452,
            'via3_x': 315,
            'via3_y': 381,
            'via4_x': 463,
            'via4_y': 350,
            'via5_x': 573,
            'via5_y': 286,
            'via6_x': 269,
            'via6_y': 246,
            'via7_x': 471,
            'via7_y': 246,
            'via8_x': 558,
            'via8_y': 404,
            'via9_x': 428,
            'via9_y': 303,
            'output_csv': os.path.join(pkg_dir,'path.csv'),
        }]
    )
    
    return LaunchDescription([
        planner_node
    ])