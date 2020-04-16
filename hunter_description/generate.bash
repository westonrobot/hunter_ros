#!/bin/bash

cd urdf
rosrun xacro xacro -o hunter_v1.urdf hunter_v1.xacro
python -m urdf2webots.importer --input=hunter_v1.urdf --output=../proto/HunterV1.proto