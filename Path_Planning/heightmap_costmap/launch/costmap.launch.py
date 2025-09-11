#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('heightmap_costmap')
    
    # Path to maps
    heightmap = os.path.join(pkg_dir, 'maps', 'heightmap.png')
    
    # Heightmap converter node
    heightmap_node = Node(
        package='heightmap_costmap',
        executable='heightmap_to_costmap',
        name='heightmap_converter',
        output='screen',
        parameters=[{
            'image_path': heightmap,
            'resolution': 0.05,
            'origin_x': 0.0,
            'origin_y': 0.0,
            'gradient_scale': 500.0,
            'stability_scale': 300.0,
        }]
    )
    
    return LaunchDescription([
        heightmap_node
    ])