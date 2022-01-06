"""Launch realsense2_camera node. Include raibo-smd specific parameters from the rs_launch default"""
import copy
import os
import sys
import pathlib
import yaml
import launch_ros.actions

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


# adding config data from .yaml file
for name,data in (config['rs_config'].items()):
    local_parameters.append({'name':name,'default':data['default'],'description':data['description']})

# additional parameters that need to be configured (params that are not relevant to .yaml file)
local_parameters +=[
    # {'name': 'enable_sync',       'default': 'true',        'description': 'thats the sync'},
    {'name': 'pointcloud.stream_index_filter'   ,   'default': "0", 'description': 'pcl stream index filter'                                        },
    {'name': 'stereo_module.inter_cam_sync_mode',   'default': "1", 'description': 'camera sync mode (0: default,1: master,2: slave,3: full slave,)'},
]

for data in local_parameters:
    for index,d in enumerate(full_parameters):
        if d['name'] == data['name']:
            d['default']     = data['default']
            d['description'] = data['description']
            break
        if index == len(full_parameters) - 1:
            full_parameters.append(data)
            break
# print(local_parameters)
for d in full_parameters:
    print(str(d))

def generate_launch_description():
    log_level = 'info'
    return LaunchDescription(rs_launch.declare_configurable_parameters(full_parameters) + [
        # Realsense
        launch_ros.actions.Node(
            package     = 'realsense2_camera',
            namespace   = LaunchConfiguration("camera_name"),
            name        = LaunchConfiguration("camera_name"),
            executable  = 'realsense2_camera_node',
            parameters  = [rs_launch.set_configurable_parameters(full_parameters)],
            output      = 'screen',
            arguments   = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty = True,
        ), # adding custom config .json file is not included here
    ])