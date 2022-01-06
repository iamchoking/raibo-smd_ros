import os
import sys
import pathlib
import yaml

with open(str(pathlib.Path(__file__).parent.absolute())+"/config.yaml", "r") as stream:
    config = yaml.safe_load(stream)

a = ['a','b','c']

for i,item in enumerate(a):
    print ("i: "+ str(i))
    print ("item: "+ str(item))