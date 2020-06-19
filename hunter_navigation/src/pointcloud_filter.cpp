/*
 * pointcloud_filter.cpp
 *
 * Created on: Jun 19, 2020 16:07
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "hunter_navigation/pointcloud_filter.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>

namespace westonrobot {
PointCloudFilter::PointCloudFilter(ros::NodeHandle* nh, std::string input,
                                   std::string output)
    : nh_(nh) {
  pc2_sub_ = nh_->subscribe(input, 1000,
                            &PointCloudFilter::LidarPointCloudCallback, this);
  pc2_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(output, 1000);
}

void PointCloudFilter::LidarPointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // convert ROS msg to PCL msg
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_data(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *pc_data);

  // filter pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  if (!pc_data->is_dense) {
    filtered_cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*pc_data, *filtered_cloud, indices);
  }

  // publish to ROS
  sensor_msgs::PointCloud2 cloud_publish;
  pcl::toROSMsg(*filtered_cloud, cloud_publish);
  pcl::toROSMsg(*pc_data, cloud_publish);
  pc2_pub_.publish(cloud_publish);
}
}  // namespace westonrobot