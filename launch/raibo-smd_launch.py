"""Launch realsense2_camera node. Include raibo-smd specific parameters from the rs_launch default"""
import copy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
rs2_dir = get_package_share_directory('realsense2_camera')
rs2_launch_dir = rs2_dir+"/launch"

pack_dir = get_package_share_directory('raibo-smd_ros')
sys.path.append(rs2_dir+"/launch")
import rs_launch

import yaml

with open(str(pathlib.Path(__file__).parent.absolute())+"/config.yaml", "r") as stream:
    config = yaml.safe_load(stream)

print(config['rs_config'])

local_parameters = [
    {'name': 'enable_pointcloud', 'default': 'true',            'description': 'enable pointcloud'},
]

def generate_launch_description():
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rs2_launch_dir, '/rs_launch.py']),
            launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
        ),
    ])