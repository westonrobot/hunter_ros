/*
 * hunter_messenger.cpp
 *
 * Created on: Jun 01, 2020 15:25
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "hunter_base/hunter_messenger.hpp"

#include <tf/transform_broadcaster.h>

#include "hunter_base/hunter_messenger.hpp"
#include "hunter_msgs/HunterStatus.h"

namespace wescore {
HunterROSMessenger::HunterROSMessenger(ros::NodeHandle *nh)
    : hunter_(nullptr), nh_(nh) {}

HunterROSMessenger::HunterROSMessenger(HunterBase *hunter, ros::NodeHandle *nh)
    : hunter_(hunter), nh_(nh) {}

void HunterROSMessenger::SetupSubscription() {
  // odometry publisher
  odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_frame_, 50);
  status_publisher_ =
      nh_->advertise<hunter_msgs::HunterStatus>("/hunter_status", 10);

  // cmd subscriber
  motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &HunterROSMessenger::TwistCmdCallback, this);
}

void HunterROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  if (!simulated_robot_) {
    hunter_->SetMotionCommand(msg->linear.x, msg->angular.z);
  } else {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    current_twist_ = *msg.get();
  }
  // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
}

void HunterROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                   double &angular) {
  std::lock_guard<std::mutex> guard(twist_mutex_);
  linear = current_twist_.linear.x;
  angular = current_twist_.angular.z;
}

void HunterROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = hunter_->GetHunterState();

  // publish hunter state message
  hunter_msgs::HunterStatus status_msg;

  status_msg.header.stamp = current_time_;

  status_msg.linear_velocity = state.linear_velocity;
  status_msg.angular_velocity = state.angular_velocity;

  status_msg.base_state = state.base_state;
  status_msg.control_mode = state.control_mode;
  status_msg.fault_code = state.fault_code;
  status_msg.battery_voltage = state.battery_voltage;

  for (int i = 0; i < 3; ++i) {
    status_msg.motor_states[i].current = state.motor_states[i].current;
    status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
    status_msg.motor_states[i].temperature = state.motor_states[i].temperature;
  }

  status_publisher_.publish(status_msg);

  // publish odometry and tf
  PublishOdometryToROS(state.linear_velocity, state.angular_velocity, dt);

  // record time for next integration
  last_time_ = current_time_;
}

void HunterROSMessenger::PublishSimStateToROS(double linear, double angular) {
  current_time_ = ros::Time::now();
  double dt = 1.0 / sim_control_rate_;

  // publish hunter state message
  hunter_msgs::HunterStatus status_msg;

  status_msg.header.stamp = current_time_;

  status_msg.linear_velocity = linear;
  status_msg.angular_velocity = angular;

  status_msg.base_state = 0x00;
  status_msg.control_mode = 0x01;
  status_msg.fault_code = 0x00;
  status_msg.battery_voltage = 29.5;

  // for (int i = 0; i < 3; ++i)
  // {
  //     status_msg.motor_states[i].current = state.motor_states[i].current;
  //     status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
  //     status_msg.motor_states[i].temperature =
  //     state.motor_states[i].temperature;
  // }

  status_publisher_.publish(status_msg);

  // publish odometry and tf
  PublishOdometryToROS(linear, angular, dt);
}

void HunterROSMessenger::PublishOdometryToROS(double linear, double angular,
                                              double dt) {
  // perform numerical integration to get an estimation of pose
  linear_speed_ = linear;
  steering_angle_ = angular;

  // propagate model model
  asc::state_t state =
      model_.Propagate({position_x_, position_y_, theta_},
                       {linear_speed_, steering_angle_}, 0, dt, dt / 100);
  position_x_ = state[0];
  position_y_ = state[1];
  theta_ = state[2];

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  // publish tf transformation
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time_;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.child_frame_id = base_frame_;

  tf_msg.transform.translation.x = position_x_;
  tf_msg.transform.translation.y = position_y_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_quat;

  tf_broadcaster_.sendTransform(tf_msg);

  // publish odometry and tf messages
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = linear_speed_;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = steering_angle_;

  odom_publisher_.publish(odom_msg);
}
}  // namespace wescore