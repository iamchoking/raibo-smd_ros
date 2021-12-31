import os
import sys
import pathlib
import yaml

with open(str(pathlib.Path(__file__).parent.absolute())+"/config.yaml", "r") as stream:
    config = yaml.safe_load(stream)

# print(config)

use_camera = 'd435i_R'

print(pharse_tf_args(config, use_camera))