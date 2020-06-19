/*
 * pointcloud_filter.hpp
 *
 * Created on: Jun 19, 2020 15:47
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef POINTCLOUD_FILTER_HPP
#define POINTCLOUD_FILTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace westonrobot {
class PointCloudFilter {
 public:
  PointCloudFilter(ros::NodeHandle* nh, std::string input, std::string output);

 private:
  ros::NodeHandle* nh_;
  ros::Subscriber pc2_sub_;
  ros::Publisher pc2_pub_;

  void LidarPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};
}  // namespace westonrobot

#endif /* POINTCLOUD_FILTER_HPP */
