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

# vel_dir        = get_package_share_directory('velodyne')
# vel_launch_dir = os.path.join(vel_dir,"launch")

pkg_dir        = get_package_share_directory('raibo-smd_ros')
pkg_launch_dir = os.path.join(pkg_dir,"launch")

sys.path.append(pkg_launch_dir)
# sys.path.append(vel_launch_dir)

with open(os.path.join(pkg_launch_dir,'config.yaml')) as stream:
    config = yaml.safe_load(stream)

def generate_launch_description():
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'full.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
    )

    return LaunchDescription(
        [
        DeclareLaunchArgument('enable_rviz',default_value="true",description="run rviz node"),
        rviz_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_launch_dir, '/lidar.py']),
            launch_arguments={'enable_rviz':'false'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_launch_dir, '/head_pcl_mult.py']),
            launch_arguments={'enable_rviz':'false'}.items(),
        ),
    ])