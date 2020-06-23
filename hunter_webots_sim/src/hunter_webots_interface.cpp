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

#include <webots_ros/get_float.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <ros/service.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>

namespace westonrobot {
HunterWebotsInterface::HunterWebotsInterface(ros::NodeHandle *nh,
                                             HunterROSMessenger *msger,
                                             uint32_t time_step)
    : nh_(nh), messenger_(msger), time_step_(time_step) {}

void HunterWebotsInterface::InitComponents(std::string controller_name) {
  // reset controller name
  robot_name_ = controller_name;

  // setup robot actuators
  SetupRobot();

  // setup additional sensors
  SetupLidar();
  SetupIMU();
}

void HunterWebotsInterface::SetupRobot() {
  pc_sub_ =
      nh_->subscribe(robot_name_ + "/rslidar/point_cloud", 1000,
                     &HunterWebotsInterface::LidarNewPointCloudCallback, this);
  pc2_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("/rslidar_points", 1000);

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

void HunterWebotsInterface::SetupLidar() {
  std::string lidar_enable_srv_name = robot_name_ + "/rslidar/enable";
  if (ros::service::exists(lidar_enable_srv_name, true)) {
    // enable lidar
    ros::ServiceClient enable_lidar_client;
    webots_ros::set_int enable_lidar_srv;
    enable_lidar_client =
        nh_->serviceClient<webots_ros::set_int>(lidar_enable_srv_name);
    enable_lidar_srv.request.value = 10;
    if (enable_lidar_client.call(enable_lidar_srv) &&
        enable_lidar_srv.response.success == 1)
      ROS_INFO("Lidar Enabled.");
    else
      ROS_ERROR("Failed to enable Lidar");

    // enable lidar pointcloud
    std::string lidar_enable_pc_srv_name =
        robot_name_ + "/rslidar/enable_point_cloud";
    ros::ServiceClient enable_lidar_pc_client;
    webots_ros::set_bool enable_lidar_pc_srv;
    enable_lidar_pc_client =
        nh_->serviceClient<webots_ros::set_bool>(lidar_enable_pc_srv_name);
    enable_lidar_pc_srv.request.value = true;
    if (enable_lidar_pc_client.call(enable_lidar_pc_srv) &&
        enable_lidar_pc_srv.response.success == 1)
      ROS_INFO("Lidar Pointcloud Enabled.");
    else
      ROS_ERROR("Failed to enable Lidar Pointcloud");
  }

  // publish tf
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "rslidar";
  static_transformStamped.child_frame_id = robot_name_ + "/rslidar";
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster_.sendTransform(static_transformStamped);
}

void HunterWebotsInterface::SetupIMU() {
  //   gyro_sub_;
  //   accel_sub_;
  //   imu_pub_;

  gyro_sub_ = nh_->subscribe(robot_name_ + "/gyro/values", 1000,
                             &HunterWebotsInterface::GyroNewDataCallback, this);
  accel_sub_ =
      nh_->subscribe(robot_name_ + "/accel/values", 1000,
                     &HunterWebotsInterface::AccelNewDataCallback, this);
  imu_pub_ = nh_->advertise<sensor_msgs::Imu>("/imu", 1000);

  std::string gyro_enable_srv_name = robot_name_ + "/gyro/enable";
  std::string accel_enable_srv_name = robot_name_ + "/accel/enable";

  if (ros::service::exists(gyro_enable_srv_name, true)) {
    // enable gyro
    ros::ServiceClient enable_gyro_client;
    webots_ros::set_int enable_gyro_srv;
    enable_gyro_client =
        nh_->serviceClient<webots_ros::set_int>(gyro_enable_srv_name);
    enable_gyro_srv.request.value = 100;
    if (enable_gyro_client.call(enable_gyro_srv) &&
        enable_gyro_srv.response.success == 1)
      ROS_INFO("Gyro Enabled.");
    else
      ROS_ERROR("Failed to enable Gyro");

    // enable accel
    ros::ServiceClient enable_accel_client;
    webots_ros::set_int enable_accel_srv;
    enable_accel_client =
        nh_->serviceClient<webots_ros::set_int>(accel_enable_srv_name);
    enable_accel_srv.request.value = 100;
    if (enable_accel_client.call(enable_accel_srv) &&
        enable_accel_srv.response.success == 1)
      ROS_INFO("Gyro Enabled.");
    else
      ROS_ERROR("Failed to enable Gyro");
  }

  // publish tf
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "imu_link";
  static_transformStamped.child_frame_id = robot_name_ + "/imu";
  static_transformStamped.transform.translation.x = 0.32;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0.18;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster_.sendTransform(static_transformStamped);
}

void HunterWebotsInterface::UpdateSimState() {
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
      (wheel_speed_or_position[2] + wheel_speed_or_position[3]) / 2.0 *
      HunterParams::wheel_radius;
  double steering_angle;
  if (std::abs(wheel_speed_or_position[0]) < 0.005 ||
      std::abs(wheel_speed_or_position[1]) < 0.005) {
    steering_angle = 0.0;
  } else if (wheel_speed_or_position[0] > 0) {
    // left turn (inner wheel is left wheel)
    steering_angle = std::atan(
        l / (l / std::tan(std::abs(wheel_speed_or_position[1])) + w / 2.0));
  } else if (wheel_speed_or_position[0] < 0) {
    // right turn (inner wheel is right wheel)
    steering_angle = std::atan(
        l / (l / std::tan(std::abs(wheel_speed_or_position[0])) + w / 2.0));
    steering_angle = -steering_angle;
  }
  //   std::cerr << "linear: " << linear_speed << " , angular: " <<
  //   steering_angle << std::endl;
  messenger_->PublishSimStateToROS(linear_speed, steering_angle);

  /*--------------------------------------------------------------------------------*/

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

  double wheel_cmds[4] = {0};

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
  //   std::cout << "angular: " << angular << " wheel0: " << wheel_cmds[0]
  //             << " wheel1: " << wheel_cmds[1] << std::endl;

  for (int i = 0; i < 2; ++i) {
    if (wheel_cmds[i] > 0.785) wheel_cmds[i] = 0.785;
    if (wheel_cmds[i] < -0.785) wheel_cmds[i] = -0.785;
  }

  double r = l / tan(std::abs(angular));
  double vel_o = linear * (r + w / 2.0) / r;
  double vel_i = linear * (r - w / 2.0) / r;
  double vel_l, vel_r;
  if (angular > 0.05) {
    // left turn
    vel_l = vel_i;
    vel_r = vel_o;
  } else if (angular < -0.05) {
    // right turn
    vel_r = vel_i;
    vel_l = vel_o;
  } else {
    vel_r = linear;
    vel_l = linear;
  }

  wheel_cmds[2] = vel_l / HunterParams::wheel_radius;
  wheel_cmds[3] = vel_r / HunterParams::wheel_radius;
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
  //   std::cout << "angular: " << wheel_cmds[0] << " , " << wheel_cmds[1]
  //             << " linear: " << wheel_cmds[2] << " , " << wheel_cmds[3]
  //             << std::endl;
}

void HunterWebotsInterface::GyroNewDataCallback(
    const sensor_msgs::Imu::ConstPtr &msg) {
  sensor_msgs::Imu imu_msg;
  imu_msg = *msg;
  imu_msg.header.frame_id = "imu_link";
  imu_msg.linear_acceleration = accel_data_.linear_acceleration;
  imu_msg.linear_acceleration_covariance =
      accel_data_.linear_acceleration_covariance;
  imu_pub_.publish(imu_msg);
}

void HunterWebotsInterface::AccelNewDataCallback(
    const sensor_msgs::Imu::ConstPtr &msg) {
  accel_data_ = *msg;
}

void HunterWebotsInterface::LidarNewPointCloudCallback(
    const sensor_msgs::PointCloud::ConstPtr &msg) {
  sensor_msgs::PointCloud2 pc2_msg;
  sensor_msgs::convertPointCloudToPointCloud2(*msg.get(), pc2_msg);

  // transform pointcloud
  Eigen::Matrix4f transform;
  Eigen::Quaternionf quat = Eigen::Quaternionf{
      Eigen::AngleAxisf{M_PI / 2.0, Eigen::Vector3f{1, 0, 0}}};
  transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
  transform(3, 0) = 0;
  transform(3, 1) = 0;
  transform(3, 2) = 0;
  transform(3, 3) = 1;
  sensor_msgs::PointCloud2 pc_transformed;
  pcl_ros::transformPointCloud(transform, pc2_msg, pc_transformed);

  //   sensor_msgs::PointCloud2 cloud_publish;
  //   pcl::toROSMsg(*pc_transformed, cloud_publish);
  //   cloud_publish.header = pc2_msg.header;

  // publish to ROS
  pc2_pub_.publish(pc_transformed);
  //   pc2_pub_.publish(pc2_msg);
}

}  // namespace westonrobot