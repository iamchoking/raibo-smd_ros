# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# DESCRIPTION #
# ----------- #
# Use this launch file to launch realsense2_camera node and rviz2 to view the published pointcloud.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# command line example:
# ros2 launch realsense2_camera demo_pointcloud_launch.py 

"""Launch realsense2_camera node."""
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
pack_dir = get_package_share_directory('raibo-smd_ros')
sys.path.append(rs2_dir+"/launch")
import rs_launch

import yaml

with open(str(pathlib.Path(__file__).parent.absolute())+"/config.yaml", "r") as stream:
    config = yaml.safe_load(stream)

def pharse_tf_args(config,use_camera):

    use_trans = config[use_camera]['translation_base_to_link']['data']
    trans_args = list(map(str,use_trans))

    use_zyx   = config[use_camera]['ZYX_base_to_link']['data']
    zyx_args = list(map(str,use_zyx))

    use_args = trans_args+zyx_args+['base',use_camera+'_link']

    return use_args

use_camera = 'd435i_L'
use_serial_param = "'%s'"%(config[use_camera]['serial']['data'])

local_parameters = [
    {'name': 'camera_name',       'default': use_camera,          'description': 'camera unique name'},
    # {'name': 'enable_pointcloud', 'default': 'true',            'description': 'enable pointcloud'},
    {'name': 'serial_no',         'default': use_serial_param,  'description': 'choose device by serial number'},
]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def generate_launch_description():
    rviz_config_dir = os.path.join(pack_dir, 'rviz', 'pointcloud.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )

    tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = pharse_tf_args(config,use_camera),
        )

    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pack_dir+"/launch", '/raibo-smd_launch.py']),
                launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
                ),
            rviz_node,
            tf_node
        ])