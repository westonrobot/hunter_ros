/*
 * hunter_webots_interface.cpp
 *
 * Created on: Jun 02, 2020 12:51
 * Description:
 *
 * Reference:
 * [1] https://www.xarg.org/book/kinematics/ackerman-steering/
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "hunter_webots_sim/hunter_webots_interface.hpp"

#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

// #include "hunter_webots_sim/hunter_sim_params.hpp"

namespace wescore {
HunterWebotsInterface::HunterWebotsInterface(ros::NodeHandle *nh,
                                             HunterROSMessenger *msger,
                                             uint32_t time_step)
    : nh_(nh), messenger_(msger), time_step_(time_step) {}

void HunterWebotsInterface::InitComponents(std::string controller_name) {
  // reset controller name
  robot_name_ = controller_name;

  // init motors
  for (int i = 0; i < 2; ++i) {
    // position
    webots_ros::set_float set_position_srv;
    ros::ServiceClient set_position_client =
        nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/set_position"));

    set_position_srv.request.value = 0;
    if (set_position_client.call(set_position_srv) &&
        set_position_srv.response.success)
      ROS_INFO("Position set to 0 for motor %s.", motor_names_[i].c_str());
    else
      ROS_ERROR("Failed to call service set_position on motor %s.",
                motor_names_[i].c_str());

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = nh_->serviceClient<webots_ros::set_float>(
        robot_name_ + "/" + std::string(motor_names_[i]) +
        std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.8;
    if (set_velocity_client.call(set_velocity_srv) &&
        set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.8 for motor %s.", motor_names_[i].c_str());
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
  }

  for (int i = 2; i < 4; ++i) {
    // position
    webots_ros::set_float set_position_srv;
    ros::ServiceClient set_position_client =
        nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY;
    if (set_position_client.call(set_position_srv) &&
        set_position_srv.response.success)
      ROS_INFO("Position set to INFINITY for motor %s.",
               motor_names_[i].c_str());
    else
      ROS_ERROR("Failed to call service set_position on motor %s.",
                motor_names_[i].c_str());

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = nh_->serviceClient<webots_ros::set_float>(
        robot_name_ + "/" + std::string(motor_names_[i]) +
        std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
    if (set_velocity_client.call(set_velocity_srv) &&
        set_velocity_srv.response.success == 1)
      ROS_INFO("Velocity set to 0.0 for motor %s.", motor_names_[i].c_str());
    else
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
  }
}

void HunterWebotsInterface::UpdateSimState() {
  // constants for calculation
  constexpr double rotation_radius =
      std::hypot(HunterParams::wheelbase / 2.0, HunterParams::track / 2.0) *
      2.0;
  constexpr double rotation_theta =
      std::atan2(HunterParams::wheelbase, HunterParams::track);

  // update robot state
  double wheel_speed_or_position[4];
  for (int i = 0; i < 2; ++i) {
    webots_ros::get_float get_position_srv;
    ros::ServiceClient get_position_client =
        nh_->serviceClient<webots_ros::get_float>(
            robot_name_ + "/" + std::string(motor_names_[i]) +
            std::string("/get_target_position"));

    if (get_position_client.call(get_position_srv)) {
      wheel_speed_or_position[i] = get_position_srv.response.value;
      //   ROS_INFO("Position set to %s for motor %d.", motor_names_[i].c_str(),
      //   i);
    } else
      ROS_ERROR("Failed to call service get_position on motor %s.",
                motor_names_[i].c_str());
  }
  for (int i = 2; i < 4; ++i) {
    webots_ros::get_float get_velocity_srv;
    ros::ServiceClient get_velocity_client =
        nh_->serviceClient<webots_ros::get_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/get_velocity"));

    if (get_velocity_client.call(get_velocity_srv)) {
      wheel_speed_or_position[i] = get_velocity_srv.response.value;
      //   ROS_INFO("Velocity set to 0.0 for motor %s.",
      //   motor_names_[i].c_str());
    } else
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
  }

  double linear_speed =
      (wheel_speed_or_position[2] + wheel_speed_or_position[3]) / 2.0 * HunterParams::wheel_radius;
  double steering_angle;
  if (std::abs(wheel_speed_or_position[0]) < 0.005 || std::abs(wheel_speed_or_position[1]) < 0.005) {
    steering_angle = 0.0;
  } else if (wheel_speed_or_position[0] > 0) {
    // left turn (inner wheel is left wheel)
    steering_angle =
        std::atan(l / (l / std::tan(std::abs(wheel_speed_or_position[1])) + w / 2.0));
  } else if (wheel_speed_or_position[0] < 0) {
    // right turn (inner wheel is right wheel)
    steering_angle =
        std::atan(l / (l / std::tan(std::abs(wheel_speed_or_position[0])) + w / 2.0));
  }
  messenger_->PublishSimStateToROS(linear_speed, steering_angle);

  // send robot command
  double linear, angular;
  messenger_->GetCurrentMotionCmdForSim(linear, angular);

  if (linear > HunterParams::max_linear_speed)
    linear = HunterParams::max_linear_speed;
  if (linear < -HunterParams::max_linear_speed)
    linear = -HunterParams::max_linear_speed;

  if (angular > HunterParams::max_steer_angle)
    angular = HunterParams::max_steer_angle;
  if (angular < -HunterParams::max_steer_angle)
    angular = -HunterParams::max_steer_angle;

  double wheel_cmds[4];

  // steering wheel angle calculation according to Ackermann constraint
  double theta = std::abs(angular);
  if (angular > 0) {
    // left turn
    wheel_cmds[0] = std::atan((2 * l * std::sin(theta)) /
                              (2 * l * std::cos(theta) + w * std::sin(theta)));
    wheel_cmds[1] = std::atan((2 * l * std::sin(theta)) /
                              (2 * l * std::cos(theta) - w * std::sin(theta)));

  } else if (angular < 0) {
    // right turn
    wheel_cmds[0] = -(-std::atan2(2 * l * std::cos(theta) - w * std::sin(theta),
                                  2 * l * std::sin(theta)) +
                      M_PI / 2.0);
    wheel_cmds[1] = -(-std::atan2(2 * l * std::cos(theta) + w * std::sin(theta),
                                  2 * l * std::sin(theta)) +
                      M_PI / 2.0);
  } else {
    wheel_cmds[0] = 0;
    wheel_cmds[1] = 0;
  }
  std::cout << "angular: " << angular << " wheel0: " << wheel_cmds[0]
            << " wheel1: " << wheel_cmds[1] << std::endl;

  for (int i = 0; i < 2; ++i) {
    if (wheel_cmds[i] > 0.785) wheel_cmds[i] = 0.785;
    if (wheel_cmds[i] < -0.785) wheel_cmds[i] = -0.785;
  }

  wheel_cmds[2] = linear / HunterParams::wheel_radius;
  wheel_cmds[3] = linear / HunterParams::wheel_radius;
  // set steering angle
  for (int i = 0; i < 2; ++i) {
    webots_ros::set_float set_position_srv;
    ros::ServiceClient set_position_client =
        nh_->serviceClient<webots_ros::set_float>(robot_name_ + "/" +
                                                  std::string(motor_names_[i]) +
                                                  std::string("/set_position"));

    set_position_srv.request.value = wheel_cmds[i];
    if (set_position_client.call(set_position_srv) &&
        set_position_srv.response.success) {
      //   ROS_INFO("Position set to %f for motor %s.", wheel_cmds[i],
      //            motor_names_[i].c_str());
    } else {
      ROS_ERROR("Failed to call service set_position on motor %s.",
                motor_names_[i].c_str());
    }
  }
  // set driving velocity
  for (int i = 2; i < 4; ++i) {
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = nh_->serviceClient<webots_ros::set_float>(
        robot_name_ + "/" + std::string(motor_names_[i]) +
        std::string("/set_velocity"));

    set_velocity_srv.request.value = wheel_cmds[i];
    if (set_velocity_client.call(set_velocity_srv) &&
        set_velocity_srv.response.success == 1) {
      //   ROS_INFO("Velocity set to 0.0 for motor %s.",
      //   motor_names_[i].c_str());
    } else {
      ROS_ERROR("Failed to call service set_velocity on motor %s.",
                motor_names_[i].c_str());
    }
  }
}

}  // namespace wescore