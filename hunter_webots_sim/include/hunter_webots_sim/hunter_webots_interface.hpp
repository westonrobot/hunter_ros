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

#include <string>

#include "hunter_base/hunter_messenger.hpp"

namespace wescore {
class HunterWebotsInterface {
 public:
  HunterWebotsInterface(ros::NodeHandle* nh, HunterROSMessenger* msger,
                        uint32_t time_step);

  void InitComponents(std::string controller_name);
  void UpdateSimState();

 private:
  ros::NodeHandle* nh_;
  HunterROSMessenger* messenger_;
  uint32_t time_step_;

  std::string robot_name_ = "hunter_v1";
  const std::vector<std::string> motor_names_{
      "front_right_steering", "front_left_steering", "rear_left_wheel",
      "rear_right_wheel"};
};
}  // namespace wescore

#endif /* HUNTER_WEBOTS_INTERFACE_HPP */
