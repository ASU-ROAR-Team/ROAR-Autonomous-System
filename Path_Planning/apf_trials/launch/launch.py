#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('apf_trials')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # APF node
    apf_node = Node(
        package='apf_trials',
        executable='APF_update',
        name='APF_update',
        parameters=[params_file],
        output='screen'
    )
    
    return LaunchDescription([
        apf_node
    ])