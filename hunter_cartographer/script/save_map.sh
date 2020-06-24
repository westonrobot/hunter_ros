#!/bin/bash

echo "Map name ([ENTER] to finish): "

read mapname

rosservice call /finish_trajectory 0
rosservice call /write_state "filename: '../maps/$mapname.pbstream'
include_unfinished_submaps: false" 

rosrun map_server map_saver -f $mapname