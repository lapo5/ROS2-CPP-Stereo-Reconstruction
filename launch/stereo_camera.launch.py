from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import sys
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    params_allied_left = os.path.join(
        get_package_share_directory("stereo_logger"),
        "params",
        "params_cam_left.yaml",
    )
    params_allied_right = os.path.join(
        get_package_share_directory("stereo_logger"),
        "params",
        "params_cam_right.yaml",
    )

    node_cam_right = Node(
        package="hal_allied_vision_camera",
        executable="av_node",
        name="av_node_right",
        parameters=[params_allied_right],
    )

    node_cam_left = Node(
        package="hal_allied_vision_camera",
        executable="av_node",
        name="av_node_left",
        parameters=[params_allied_left],
    )

    return LaunchDescription([node_cam_right, node_cam_left])
