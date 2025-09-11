#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('heightmap_costmap')
    
    # Path to maps
    calibration_map = os.path.join(pkg_dir, 'maps', 'Calibration_map.png')
    
    # Calibration node
    calibration_node = Node(
        package='heightmap_costmap',
        executable='map_calibration',
        name='calibration_node',
        output='screen',
        parameters=[{
            'image_path': calibration_map,
            'real_coords': '[[0.0,0.0 ], [10.0126, 0.0 ], [20.127, 0.0]]',
            'input_csv': os.path.join(pkg_dir, 'path.csv'),
            'output_csv': os.path.join(pkg_dir, 'real_path.csv'),
        }]
    )
    
    return LaunchDescription([
        calibration_node
    ])