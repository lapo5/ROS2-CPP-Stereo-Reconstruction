import os
import yaml

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    params_stereo_logger = os.path.join(
        get_package_share_directory("stereo_logger"),
        "params",
        "params_stereo_logger.yaml",
    )
    
    node_stereo_logger = Node(
        package="stereo_logger",
        executable="stereo_logger",
        name="stereo_logger",
        parameters=[params_stereo_logger],
    )
    

    return LaunchDescription([node_stereo_logger])
