#!/bin/bash

rosservice call /finish_trajectory 0
rosservice call /write_state "filename: '/home/hunter/Workspace/maps/testmap.pbstream' include_unfinished_submaps: false"