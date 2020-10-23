# ROS Packages for Hunter Mobile Base

## Packages

* hunter_base: a ROS wrapper around Hunter SDK to monitor and control the robot
* hunter_bringup: launch and configuration files to start ROS nodes 
* hunter_msgs: hunter related message definitions

## Communication interface setup

Please refer to the [README](https://github.com/westonrobot/wrp_sdk#hardware-interface) of "wrp_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS package

If you're using ROS Kinetic in Ubuntu 16.04, please refer to instructions [here](https://apt.kitware.com/) to install the latest version of CMake. A version later than 3.10.2 is required.

1. Install dependent packages

    ```
    $ sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
    ```
    
2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone --recursive https://github.com/westonrobot/ugv_sdk.git
    $ git clone https://github.com/westonrobot/hunter_base.git
    $ cd ..
    $ catkin_make
    ```

3. Launch ROS nodes
 
* Start the base node for the real robot

    ```
    $ roslaunch hunter_bringup hunter_robot_base.launch
    ```
    
**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
