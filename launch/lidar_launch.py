"""Launch realsense2_camera node."""
import copy
import os
import sys
import pathlib
import yaml

from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription,DeclareLaunchArgument
from launch.conditions                 import IfCondition
from launch_ros.actions                import Node
from launch.substitutions              import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages       import get_package_share_directory

vel_dir        = get_package_share_directory('velodyne')
vel_launch_dir = os.path.join(vel_dir,"launch")

pkg_dir        = get_package_share_directory('raibo-smd_ros')
pkg_launch_dir = os.path.join(pkg_dir,"launch")

sys.path.append(pkg_launch_dir)
sys.path.append(vel_launch_dir)

with open(os.path.join(pkg_launch_dir,'config.yaml')) as stream:
    config = yaml.safe_load(stream)


def pharse_tf_args(config,use_device):

    use_trans = config[use_device]['translation_base_to_link']['data']
    trans_args = list(map(str,use_trans))

    use_zyx   = config[use_device]['ZYX_base_to_link']['data']
    zyx_args = list(map(str,use_zyx))

    use_args = trans_args+zyx_args+['base',use_device]

    return use_args

def generate_launch_description():
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'lidar.rviz')
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
        name="base_to_"+'velodyne',
        arguments = pharse_tf_args(config,'velodyne'),
    )

    return LaunchDescription(
        [
        tf_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vel_launch_dir, '/velodyne-all-nodes-VLP16-launch.py']),
            # launch_arguments=set_configurable_parameters(params1).items(),
        ),
        DeclareLaunchArgument('enable_rviz',default_value="true",description="run rviz node"),
        rviz_node,
    ])