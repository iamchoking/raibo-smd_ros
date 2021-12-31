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
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=my_D435 device_type1:=d435 camera_name2:=my_d415 device_type2:=d415 

"""Launch realsense2_camera node."""
import copy
import launch_ros.actions
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

sys.path.append(rs2_launch_dir)
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

R_camera = "d435i_R"
L_camera = "d435i_L"

R_serial_param = "'%s'"%(config[R_camera]['serial']['data'])
L_serial_param = "'%s'"%(config[L_camera]['serial']['data'])

local_parameters = [
    {'name': 'camera_name1',       'default': R_camera,       'description': 'camera unique name'},
    {'name': 'camera_name2',       'default': L_camera,       'description': 'camera unique name'},
    {'name': 'serial_no1',         'default': R_serial_param, 'description': 'choose device by serial number'},
    {'name': 'serial_no2',         'default': L_serial_param, 'description': 'choose device by serial number'},
    {'name': 'enable_pointcloud1', 'default': 'true',         'description': 'enable pointcloud'},
    {'name': 'enable_pointcloud2', 'default': 'true',         'description': 'enable pointcloud'},
]

# print(type(config['d435i_R']['serial']))

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params
    
def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')

    rviz_config_dir = os.path.join(pack_dir, 'rviz', 'pointcloud_multicam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )

    tf_R_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = pharse_tf_args(config,L_camera),
        )

    tf_L_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = pharse_tf_args(config,R_camera),
        )

    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) + 
        rs_launch.declare_configurable_parameters(params2) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rs2_launch_dir, '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params1).items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rs2_launch_dir, '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params2).items(),
        ),
        rviz_node,
        tf_R_node,
        tf_L_node,
    ])