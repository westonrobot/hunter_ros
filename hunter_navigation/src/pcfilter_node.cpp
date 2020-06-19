/*
 * pcfilter_node.cpp
 *
 * Created on: Jun 19, 2020 16:27
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "hunter_navigation/pointcloud_filter.hpp"

using namespace westonrobot;

int main(int argc, char *argv[]) {
  // setup ROS node
  ros::init(argc, argv, "rslidar_pointcloud_filter");
  ros::NodeHandle node("");

  PointCloudFilter pcf(&node, "/rslidar_points", "/rslidar_points_filtered");

  ros::spin();

  return 0;
}