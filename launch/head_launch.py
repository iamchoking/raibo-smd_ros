"""Launch realsense2_camera node. Include raibo-smd specific parameters from the rs_launch default"""
import copy
import os
import sys
import pathlib
import yaml

from launch                            import LaunchDescription
from launch.actions                    import IncludeLaunchDescription
from launch_ros.actions                import Node
from launch.substitutions              import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages       import get_package_share_directory

rs2_dir        = get_package_share_directory('realsense2_camera')
rs2_launch_dir = os.path.join(rs2_dir,"launch")

pkg_dir        = get_package_share_directory('raibo-smd_ros')
pkg_launch_dir = os.path.join(pkg_dir,"launch")

sys.path.append(pkg_launch_dir)
sys.path.append(rs2_launch_dir)
import rs_launch

# with open(str(pathlib.Path(__file__).parent.absolute())+"/config.yaml", "r") as stream:
with open(os.path.join(pkg_launch_dir,'config.yaml')) as stream:
    config = yaml.safe_load(stream)

local_parameters = []
full_parameters = copy.deepcopy(rs_launch.configurable_parameters)
# for d in full_parameters:
#     print(str(d))
# print('---')
for name,data in (config['rs_config'].items()):
    local_parameters.append(
        {'name':name,'default':data['default'],'description':data['description']}
    )
    for d in full_parameters:
        if d['name'] == name:
            d['default']     = data['default']
            d['description'] = data['description']
            break
# print(local_parameters)
# for d in full_parameters:
#     print(str(d))
# print(full_parameters)    

def generate_launch_description():
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rs2_launch_dir, '/rs_launch.py']),
            launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
        ),
    ])
