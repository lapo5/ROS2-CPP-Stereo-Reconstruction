import os
import yaml

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    params_stereo_filepath = os.path.join(
        get_package_share_directory("stereo_reconstruction_cpp"),
        "params",
        "params_stereo_reconstruction_tests.yaml",
    )

    stereo_acquisition_node = Node(
        package="stereo_reconstruction_cpp",
        executable="stereo_reconstruction",
        name="stereo_reconstruction",
        parameters=[params_stereo_filepath],
    )
    

    return LaunchDescription([
        stereo_acquisition_node
    ])
