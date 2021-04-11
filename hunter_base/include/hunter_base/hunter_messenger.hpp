/*
 * hunter_messenger.hpp
 *
 * Created on: Jun 01, 2020 15:18
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_MESSENGER_HPP
#define HUNTER_MESSENGER_HPP

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <string>

// #include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ugv_sdk/hunter/hunter_base.hpp"

#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "hunter_base/bicycle_model.hpp"
#include "hunter_base/hunter_params.hpp"
// #include "hunter_base/system_propagator.hpp"

namespace westonrobot {
template <typename SystemModel>
class SystemPropagator {
 public:
  asc::state_t Propagate(asc::state_t init_state,
                         typename SystemModel::control_t u, double t0,
                         double tf, double dt) {
    double t = t0;
    asc::state_t x = init_state;

    while (t <= tf) {
      integrator_(SystemModel(u), x, t, dt);
      // Note: you may need to add additional constraints to [x]
    }

    return x;
  }

 private:
  asc::RK4 integrator_;
};

class HunterROSMessenger {
 public:
  explicit HunterROSMessenger(ros::NodeHandle *nh);
  HunterROSMessenger(HunterBase *hunter, ros::NodeHandle *nh);

  std::string odom_frame_;
  std::string base_frame_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;
  bool publish_tf_ = true;

  void SetupSubscription();
  void ResetOdometry();

  void PublishStateToROS();
  void PublishSimStateToROS(double linear, double angular);

  void GetCurrentMotionCmdForSim(double &linear, double &angular);

 private:
  HunterBase *hunter_;
  ros::NodeHandle *nh_;

  std::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;

  ros::Publisher odom_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber motion_cmd_subscriber_;
  ros::Subscriber integrator_reset_subscriber_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // control inputs
  double linear_speed_ = 0.0;
  double steering_angle_ = 0.0;

  static constexpr double l = HunterParams::wheelbase;
  static constexpr double w = HunterParams::track;
  static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees

  // state variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  SystemPropagator<BicycleKinematics> model_;

  ros::Time last_time_;
  ros::Time current_time_;

  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void ResetOdomIntegratorCallback(const std_msgs::Bool::ConstPtr &msg);
  void PublishOdometryToROS(double linear, double angular, double dt);
};
}  // namespace westonrobot

#endif /* HUNTER_MESSENGER_HPP */
