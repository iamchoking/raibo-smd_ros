"""Launch realsense2_camera node."""
import copy
import os
import sys
import pathlib
import yaml

from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription,DeclareLaunchArgument
from launch_ros.actions                import Node
from launch.substitutions              import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions                 import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages       import get_package_share_directory

rs2_dir        = get_package_share_directory('realsense2_camera')
rs2_launch_dir = os.path.join(rs2_dir,"launch")

pkg_dir        = get_package_share_directory('raibo-smd_ros')
pkg_launch_dir = os.path.join(pkg_dir,"launch")

sys.path.append(pkg_launch_dir)
sys.path.append(rs2_launch_dir)
import rs_launch

with open(os.path.join(pkg_launch_dir,'config.yaml')) as stream:
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
    {'name': 'camera_name',       'default': use_camera,        'description': 'camera unique name'},
    {'name': 'serial_no',         'default': use_serial_param,  'description': 'choose device by serial number'},
]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def generate_launch_description():
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'head_mult_rgb.rviz')
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        )

    tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = pharse_tf_args(config,use_camera),
        )

    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
            tf_node,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_launch_dir, '/head_launch.py']),
                launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
                ),
            DeclareLaunchArgument('enable_rviz',default_value="true",description="run rviz node"),
            rviz_node,
        ])