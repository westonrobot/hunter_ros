/*
 * hunter_webots_interface.hpp
 *
 * Created on: Jun 02, 2020 12:52
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_WEBOTS_INTERFACE_HPP
#define HUNTER_WEBOTS_INTERFACE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>

#include "hunter_base/hunter_messenger.hpp"
#include "hunter_base/hunter_params.hpp"

namespace westonrobot {
class HunterWebotsInterface {
 public:
  HunterWebotsInterface(ros::NodeHandle* nh, HunterROSMessenger* msger,
                        uint32_t time_step);

  void InitComponents(std::string controller_name);
  void UpdateSimState();

 private:
  uint32_t time_step_;
  HunterROSMessenger* messenger_;

  ros::NodeHandle* nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc2_pub_;
  ros::Subscriber gyro_sub_;
  ros::Subscriber accel_sub_;
  ros::Publisher imu_pub_;
  
  sensor_msgs::Imu accel_data_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  static constexpr double l = HunterParams::wheelbase;
  static constexpr double w = HunterParams::track;

  std::string robot_name_ = "hunter_v1";
  const std::vector<std::string> motor_names_{
      "front_right_steering", "front_left_steering", "rear_left_wheel",
      "rear_right_wheel"};

  void SetupRobot();
  void SetupLidar();
  void SetupIMU();

  void GyroNewDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void AccelNewDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void LidarNewPointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
};
}  // namespace westonrobot

#endif /* HUNTER_WEBOTS_INTERFACE_HPP */
