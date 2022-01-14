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

    bridge_node = Node(
        package = "raibo-smd_ros",
        executable = "syncer_bridge",
        name="raibo_full_bridge",
        parameters=[{'enable_head':True},{'enable_lidar':True}],
        condition=IfCondition(LaunchConfiguration("enable_raibo_bridge"))
    )

    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'raibo_smd.rviz')
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
        DeclareLaunchArgument('enable_raibo_bridge',default_value="true",description="launch bridge node"),
        bridge_node,        
        DeclareLaunchArgument('enable_rviz',default_value="true",description="run rviz node"),
        rviz_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_launch_dir, '/lidar_launch.py']),
            launch_arguments={'enable_rviz':'false','enable_raibo_bridge':'false'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_launch_dir, '/head_launch.py']),
            launch_arguments={'enable_rviz':'false','enable_raibo_bridge':'false'}.items(),
        ),
    ])