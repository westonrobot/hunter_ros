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

namespace westonrobot {
PointCloudFilter::PointCloudFilter(ros::NodeHandle *nh, std::string input, std::string output):nh_(nh) {
    pc2_sub_ =
      nh_->subscribe(input, 1000,
                     &PointCloudFilter::LidarPointCloudCallback, this);
    pc2_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(output, 1000);
}

void LidarPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    if (!points->is_dense) {
        points_filtered->is_dense = false;
        std::vector indices;
        pcl::removeNaNFromPointCloud(*points_filtered,*points_filtered, indices);
    }

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_transformed,cloud_publish);
    cloud_publish.header = input->header;

}
}