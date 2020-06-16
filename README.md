# ROS Packages for Hunter Mobile Base

## Packages

* hunter_bringup: launch and configuration files to start ROS nodes 
* hunter_base: a ROS wrapper around Hunter SDK to monitor and control the robot
* hunter_msgs: hunter related message definitions
* (hunter_ros: meta package for the Scout robot ROS packages)

## Communication interface setup

### Setup CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you also need to add a CAN transceiver for proper CAN communication. 

Please refer to this [note](https://wiki.rdu.im/_pages/Notes/Embedded-System/Linux/can-bus-in-linux.html) to learn more about the most commonly-used CAN interface commands in Linux.

### Setup CAN-To-USB adapter 

For testing, you can use the CAN-To-USB adapter to talk with the robot on any desktop/laptop:
 
1. Enable gs_usb kernel module
   
    ```
    $ sudo modprobe gs_usb
    ```

2. Bringup can device
   
   ```
   $ sudo ip link set can0 up type can bitrate 500000
   ```

3. If no error occured during the previous steps, you should be able to see the can device now by using command
   
   ```
   $ ifconfig -a
   ```

4. Install and use can-utils to test the hardware
   
    ```
    $ sudo apt install can-utils
    ```

5. Testing command
   
    ```
    # receiving data from can0
    $ candump can0
    # send data to can0
    $ cansend can0 001#1122334455667788
    ```

Two scripts inside the "hunter_bringup/scripts" folder are provided for easy setup. You can run "./setup_can2usb.bash" for the first-time setup and run "./bringup_can2usb.bash" to bring up the device each time you unplug and re-plug the adapter.

## Basic usage of the ROS package

1. Install dependent ROS packages

    ```
    $ sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
    $ sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher-gui
    $ sudo apt install ros-$ROS_DISTRO-ros-controllers
    $ sudo apt install ros-$ROS_DISTRO-webots-ros
    ```

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone -b catkin https://github.com/westonrobot/wrp_sdk.git
    $ git clone https://github.com/westonrobot/hunter_ros.git
    $ cd ..
    $ catkin_make
    ```

3. Setup Webots simulation    

* Install Webots R2020a-rev1 (download from https://cyberbotics.com/ )

* Set WEBOTS_HOME variable, add the following line to your "~/.bashrc"

    ```
    export WEBOTS_HOME=/usr/local/webots
    ```

    Adjust the path accordingly if you installed Webots to a different place.

4. Launch ROS nodes
 
* Start the base node for the real robot

    ```
    $ roslaunch hunter_bringup hunter_base.launch
    ```

* Start the Webots-based simulation

    ```
    $ roslaunch hunter_bringup hunter_base_webots_sim.launch
    ```
    
    **SAFETY PRECAUSION**: 

    The default command values of the keyboard teleop node are high, make sure you decrease the speed commands before starting to control the robot with your keyboard! Have your remote controller ready to take over the control whenever necessary. 
