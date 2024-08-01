/**
 * @file ground_truth_map.cpp
 */

#include <cstdint>
#include <cstdio>
#include <string>
#include <fstream>
#include <iostream>
#include <bitset>
#include <map>
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

#include "semantic_kitti_tools/util.hpp"
#include "semantic_kitti_tools/param.hpp"

using namespace semantic_kitti_tools;

void usage()
{
  std::cerr << "[usage] $ ros2 run semantic_kitti_tools ground_truth_map <target_sequence_dir> <calibration_dir>" << std::endl
            << "[ex] $ ros2 run semantic_kitti_tools ground_truth_map ~/semantic_kitti/05  ~/data_odometry_calib/dataset/sequence/05" << std::endl;
}

int main(int argc, char **argv)
{
  std::cout << "ground_truth_map" << std::endl;

  std::vector<std::string > pure_args = rclcpp::remove_ros_arguments(argc, argv);
  for (auto a : pure_args) std::cout << a << std::endl;
  if (pure_args.size() < 3)
  {
    usage();
    return -1;
  }
  std::string dataset_path = pure_args[1];
  std::string calib_path = pure_args[2];

  // Load pose file
  std::string pose_filepath = dataset_path + "/poses.txt";
  std::vector<Eigen::Vector3d> pos_list;
  std::vector<Eigen::Matrix3d> rot_list;
  long line_count = 0; // the number of data for the sequence
  loadPoseFromFile(pose_filepath, pos_list, rot_list, line_count);
  std::cout << line_count << " lines" << std::endl;

  // Load LiDAR external calibration file
  std::string calib_filepath = calib_path + "/calib.txt";
  Eigen::Matrix4d tr;
  LoadLiDARCalibFromFile(calib_filepath, tr);

  // Make points map by using pose file.
  pcl::PointCloud<pcl::PointXYZRGB> whole_map;
  for (int pose_i = 0; pose_i < line_count; pose_i++) // loop for each pose on sequence
  {
    if (pose_i == 0 || pose_i%(line_count/10)==0 || pose_i == line_count-1)
      std::cout << pose_i << "/" << line_count << std::endl; // check
                                                             //
    std::string point_cloud_filename = dataset_path + "/velodyne/";
    std::string label_filename= dataset_path + "/labels/";
    std::string filename = std::to_string(pose_i);
    filename = filename.insert(0, 6-filename.length(), '0');
    point_cloud_filename += filename + ".bin";
    label_filename += filename + ".label";

    // Load pointclouds & labels at the same trigger
    float **pc_data; // be deteted at last
    long number_of_points = 0;
    loadPointCloudFromFile(point_cloud_filename, pc_data, number_of_points);
    std::vector<uint16_t> labels, instance_id;
    long number_of_labels = 0;
    loadPointLabelFromFile(label_filename, labels, instance_id, number_of_labels);

    if (number_of_points != number_of_labels)
    {
      std::cerr << "the numbers of points and labels is different" << std::endl;
      // pc_data should be delete
      exit(-1);
    }

    // Make labeled-pointcloud with eliminating unused-label points
    pcl::PointCloud<pcl::PointXYZRGB> points;
    // From center(P0) to velodyne
    Eigen::Matrix<double, 4, 4> wtp;
    wtp.block(0, 0, 3, 3) = rot_list[pose_i];
    wtp.block(0, 3, 3, 1) = pos_list[pose_i];
    wtp(3, 3) = 1;
    points.reserve(number_of_points);
    for (long p_i = 0; p_i < number_of_points; p_i++)
    {
      // Remove points which label is static object such as building or load
      uint16_t label = labels[p_i];
      if (label != 40 /* road class */ && label != 50 /* building class */) continue;

      // Convert points and set color based on its label
      Eigen::Vector4d v(pc_data[p_i][0], pc_data[p_i][1], pc_data[p_i][2], 1);
      Eigen::Vector4d v_new = wtp*tr*v;
      pcl::PointXYZRGB p;
      p.x = v_new.x();
      p.y = v_new.y();
      p.z = v_new.z();
      std::array<int, 3> c;
      try
      {
        c = color_map.at(labels[p_i]);
      }
      catch(std::exception &e)
      {
        std::cerr << "Error p_i: " << p_i << "invalid label num" << labels[p_i] << std::endl;
        exit(-1);
      }
      p.r = c[0];
      p.g = c[1];
      p.b = c[2];
      points.push_back(p);
    }

    // Merge into whole map
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(points.makeShared());
    sor.setLeafSize(0.2, 0.2, 0.2);
    sor.filter(points);
    whole_map += points;
    if (pose_i%100 == 0)
    {
      pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud(whole_map.makeShared());
      sor.setLeafSize(0.2, 0.2, 0.2);
      sor.filter(whole_map);
    }

    // Delete objects
    for (int i = 0; i < number_of_points; i++)
    {
      delete[] pc_data[i];
    }
    delete[] pc_data;
  }
  sensor_msgs::msg::PointCloud2 whole_map_msg;
  pcl::toROSMsg(whole_map, whole_map_msg);
  whole_map_msg.header.frame_id = "map";

  // ROS clock
  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  // make path msg
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  auto stamp = ros_clock.now();
  path_msg.header.stamp = stamp;
  path_msg.poses.reserve(line_count);
  for (int i = 0; i < line_count; i++)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = stamp;
    ps.pose.position = tf2::toMsg(pos_list[i]);
    ps.pose.orientation = tf2::toMsg(Eigen::Quaterniond(rot_list[i]));
    path_msg.poses.push_back(ps);
  }

  // make one sample of pointcloud
  // Tr*X(velodyne) -> reference coordinate(P0, left rectified camera)
  sensor_msgs::msg::PointCloud2 pc_msg;
  {
    std::string point_cloud_filename = dataset_path + "/velodyne/000000.bin";
    float **pc_data; // Should be "malloced" in the scope where the variable is decreared
    long number_of_points = 0;
    loadPointCloudFromFile(point_cloud_filename, pc_data, number_of_points);

    std::cout << "point num: " << number_of_points << std::endl;

    std::string label_filename = dataset_path + "/labels/000000.label";

    std::vector<uint16_t> labels, instance_id;
    long number_of_labels = 0;
    loadPointLabelFromFile(label_filename, labels, instance_id, number_of_labels);


    std::cout << "make pointcloud" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> points;
    points.reserve(number_of_points);
    for (int i = 0; i < number_of_points; i++)
    {
      Eigen::Vector4d v(pc_data[i][0], pc_data[i][1], pc_data[i][2], 1);
      Eigen::Vector4d v_new = tr*v;
      pcl::PointXYZRGB p;
      p.x = v_new[0];
      p.y = v_new[1];
      p.z = v_new[2];
      std::array<int, 3> c;
      try
      {
        c = color_map.at(labels[i]);
      }
      catch(std::exception &e)
      {
        std::cout << "ERROR i: " << i << ", invalid label num : " << labels[i] << std::endl;
        exit(-1);
      }
      p.r = c[0];
      p.g = c[1];
      p.b = c[2];
      points.push_back(p);
    }
    pcl::toROSMsg(points, pc_msg);
    pc_msg.header.frame_id = "map";

    for (int i = 0; i < number_of_points; i++)
    {
      delete[] pc_data[i];
    }
    delete[]  pc_data;
  }

  // Save the map
  pcl::io::savePCDFile(
      "/tmp/kitti_map.pcd",
      whole_map_msg,
      Eigen::Vector4f::Zero(),
      Eigen::Quaternionf::Identity(),
      true/* binary_mode is true */);
  std::cout << "Saved PCD file to /tmp/kitti_map.pcd" << std::endl;

  // Publish processed data
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ground_truth_map");
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(1).transient_local());
  auto pc_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("sample_pc", rclcpp::QoS(1).transient_local());
  auto map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("points_map", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(node->get_logger(), "start groud_truth_map ros communication");
  path_pub->publish(path_msg);
  pc_pub->publish(pc_msg);
  map_pub->publish(whole_map_msg);

  rclcpp::spin(node);
  node = nullptr;
  rclcpp::shutdown();
  return 0;
}
