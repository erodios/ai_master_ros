#!/usr/bin/env python3

import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Your launch configuration here

    #get the path to the package
    package_path = get_package_share_directory('tt_table_explorer')

    parameters_file = os.path.join(package_path, 'config', 'local.yaml')

    return launch.LaunchDescription([
        Node(
            package='tt_table_explorer',
            executable='tt_table_explorer.py',
            name='tt_table_explorer',
            parameters=[parameters_file]
        ),
    ])