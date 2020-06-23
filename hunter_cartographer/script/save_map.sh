#!/bin/bash

rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${HOME}/Maps/cartographer_map.bag.pbstream', include_unfinished_submaps: 'true'}"