/*
 * @Author: your name
 * @Date: 2021-08-26 15:23:22
 * @LastEditTime: 2021-08-27 10:25:35
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /hunter_ros/hunter_base/src/hunter_base_node.cpp
 */
#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "hunter_base/hunter_messenger.hpp"

using namespace westonrobot;
std::unique_ptr<HunterRobot> robot;

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "hunter_base");
  ros::NodeHandle node(""), private_node("~");

  // fetch parameters before connecting to robot
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));

  // check protocol version
  ProtocolDectctor detector;
  detector.Connect(port_name);
  auto proto = detector.DetectProtocolVersion(5);
  if (proto == ProtocolVersion::AGX_V1) {
    std::cout << "Detected protocol: AGX_V1" << std::endl;
    robot = std::unique_ptr<HunterRobot>(
    new HunterRobot(ProtocolVersion::AGX_V1));
  } else if (proto == ProtocolVersion::AGX_V2) {
    std::cout << "Detected protocol: AGX_V2" << std::endl;
    robot = std::unique_ptr<HunterRobot>(
    new HunterRobot(ProtocolVersion::AGX_V2));
  } else {
    std::cout << "Detected protocol: UNKONWN" << std::endl;
    return -1;
  }
  if (robot == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  // instantiate a robot object
  HunterROSMessenger messenger(robot.get(), &node);


  // fetch parameters before connecting to robot
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_,
                           false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<bool>("publish_tf", messenger.publish_tf_,  true);


  // connect to robot and setup ROS subscription
  if (port_name.find("can") != std::string::npos) {
    robot->EnableCommandedMode();
    ROS_INFO("Using CAN bus to talk with the robot: %s", port_name.c_str());
  } else {
    ROS_ERROR("Only CAN bus interface is supported for now");
    return -1;
  }

  messenger.SetupSubscription();
  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate_50hz(50);  // 50Hz
  while (ros::ok()) {
    messenger.PublishStateToROS();
    ros::spinOnce();
    rate_50hz.sleep();
  }

  return 0;
}
