/**
 * @file make_rosbag2.cpp
 */

#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag2_cpp/writer.hpp>

#include "semantic_kitti_tools/util.hpp"
#include "semantic_kitti_tools/param.hpp"

int main(int argc, char **argv)
{
  
  return 0;
}

