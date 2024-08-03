/**
 * @file make_rosbag2.cpp
 * @todo Write image to bag
 */

#include <fstream>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_msgs/msg/detail/tf_message__struct.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag2_cpp/writer.hpp>
#include <rmw/qos_string_conversions.h>

#include "semantic_kitti_tools/util.hpp"
#include "semantic_kitti_tools/param.hpp"

using namespace semantic_kitti_tools;

void usage()
{
  std::cerr << "[usage] $ ros2 run semantic_kitti_tools make_rosbag2 <target_sequence_dir> <calibration_dir>" << std::endl
            << "[ex] $ ros2 run semantic_kitti_tools make_rosbag2 ~/semantic_kitti/05  ~/data_odometry_calib/dataset/sequence/05" << std::endl;

}

int main(int argc, char **argv)
{
  std::cout << "make_rosbag2" << std::endl;

  std::vector<std::string > pure_args = rclcpp::remove_ros_arguments(argc, argv);
  for (auto a : pure_args) std::cout << a << std::endl;
  if (pure_args.size() < 3)
  {
    usage();
    return -1;
  }
  std::string dataset_path = pure_args[1];
  std::string calib_path = pure_args[2];

  bool is_set_gt_pose = false;
  if (pure_args.size() > 3)
  {
    is_set_gt_pose = (pure_args[3] == "true")? true : false;
    std::cout << "Write ground trush pose TF: "
              << ((is_set_gt_pose)? "true" : "false") << std::endl;
  }

  // Load poses file
  std::string pose_filepath = dataset_path + "/poses.txt";
  std::vector<Eigen::Vector3d> pos_list;
  std::vector<Eigen::Matrix3d> rot_list;
  long poses_linecount = 0; // the number of data for the sequence
  loadPoseFromFile(pose_filepath, pos_list, rot_list, poses_linecount);

  // Load times file
  std::string times_filepath = calib_path + "/times.txt";
  std::vector<long> nanosecond_list;
  long times_linecount = 0;
  LoadTriggerTimeFromFile(times_filepath, nanosecond_list, times_linecount);

  // the number of lines for poses and times should be same
  if (poses_linecount != times_linecount)
  {
    std::cerr << "The number of lines for poses and times is different" << std::endl;
    exit(-1);
  }

  // Load LiDAR external calibration file
  std::string calib_filepath = calib_path + "/calib.txt";
  Eigen::Matrix4d tr;
  LoadLiDARCalibFromFile(calib_filepath, tr);

  // Make rosbag
  std::unique_ptr<rosbag2_cpp::Writer> writer = std::make_unique<rosbag2_cpp::Writer>();
  const std::string bagfile_path =
    std::string("/tmp/kitti_odometry_bag")+ ((is_set_gt_pose)? "_with_gt_pose" : "");
  writer->open(bagfile_path);

  // Set TF topics and its QoS profile
  rosbag2_storage::TopicMetadata tf_meta;
  tf_meta.name = "/tf";
  tf_meta.type = "tf2_msgs/msg/TFMessage";
  tf_meta.serialization_format = "cdr";
  tf_meta.offered_qos_profiles =
"- history: "+std::to_string(rmw_qos_history_policy_from_str("keep_last"))+"\n"
"  depth: 100\n"
"  reliability: "+std::to_string(rmw_qos_reliability_policy_from_str("reliable"))+"\n"
"  durability: "+std::to_string(rmw_qos_durability_policy_from_str("volatile"))+"\n"
"  deadline:\n"
"    sec: 0\n"
"    nsec: 0\n"
"  lifespan:\n"
"    sec: 0\n"
"    nsec: 0\n"
"  liveliness: 0\n"
"  liveliness_lease_duration:\n"
"    sec: 0\n"
"    nsec: 0\n"
"  avoid_ros_namespace_conventions: false";
  rosbag2_storage::TopicMetadata tf_static_meta;
  tf_static_meta.name = "/tf_static";
  tf_static_meta.type = "tf2_msgs/msg/TFMessage";
  tf_static_meta.serialization_format = "cdr";
  tf_static_meta.offered_qos_profiles =
"- history: "+std::to_string(rmw_qos_history_policy_from_str("keep_last"))+"\n"
"  depth: 1\n"
"  reliability: "+std::to_string(rmw_qos_reliability_policy_from_str("reliable"))+"\n"
"  durability: "+std::to_string(rmw_qos_durability_policy_from_str("transient_local"))+"\n"
"  deadline:\n"
"    sec: 0\n"
"    nsec: 0\n"
"  lifespan:\n"
"    sec: 0\n"
"    nsec: 0\n"
"  liveliness: 0\n"
"  liveliness_lease_duration:\n"
"    sec: 0\n"
"    nsec: 0\n"
"  avoid_ros_namespace_conventions: false";

  writer->create_topic(tf_meta);
  writer->create_topic(tf_static_meta);

  // Publish static tf
  geometry_msgs::msg::TransformStamped static_tf_msg;
  Eigen::Vector3d st_v(tr(0, 3), tr(1, 3), tr(2, 3));
  Eigen::Quaterniond  st_q(Eigen::Matrix3d(tr.block(0, 0, 3, 3)));
  tf2::toMsg(st_v, static_tf_msg.transform.translation);
  static_tf_msg.transform.rotation = tf2::toMsg(st_q);
  static_tf_msg.header.frame_id = "base_link";
  static_tf_msg.child_frame_id = "velodyne";
  static_tf_msg.header.stamp = rclcpp::Time(nanosecond_list[0]);
  //@note tf_static msg should be kept, for example, static_transform_broadcaster?
  tf2_msgs::msg::TFMessage tf2_msg;
  tf2_msg.transforms.push_back(static_tf_msg);
  writer->write<tf2_msgs::msg::TFMessage>(
      tf2_msg, "/tf_static", rclcpp::Time(nanosecond_list[0]));


  // Make bagfile with loading points and label files
  for (int t_i = 0; t_i < times_linecount; t_i++)
  {
    // Check progress
    if (t_i == 0 || t_i%(times_linecount/10)==0 || t_i == times_linecount-1)
      std::cout << t_i << "/" << times_linecount << std::endl;

    // Make filepath for points and labels such as 000000.bin
    std::string point_cloud_filename = dataset_path + "/velodyne/";
    std::string label_filename= dataset_path + "/labels/";
    std::string filename = std::to_string(t_i);
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
      for (int i = 0; i < number_of_points; i++)
      {
        delete[] pc_data[i];
      }
      delete[]  pc_data;
      exit(-1);
    }

    // Timestamp is used from times.txt
    rclcpp::Time stamp(nanosecond_list[t_i]);

    // TF from map origin to center(P0)
    if (is_set_gt_pose)
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      Eigen::Isometry3d iso(
          Eigen::Translation3d(pos_list[t_i])*Eigen::Quaterniond(rot_list[t_i]));
      tf_msg = tf2::eigenToTransform(iso);
      tf_msg.header.frame_id = "world";
      tf_msg.header.stamp = stamp;
      tf_msg.child_frame_id = "base_link";
      tf2_msgs::msg::TFMessage tf2_msg;
      tf2_msg.transforms.push_back(tf_msg);
      writer->write<tf2_msgs::msg::TFMessage>(tf2_msg, "/tf", stamp);
    }

    // Make labeled(colored)-pointcloud
    pcl::PointCloud<pcl::PointXYZRGB> points;
    points.reserve(number_of_points);
    for (long p_i = 0; p_i < number_of_points; p_i++)
    {
      // Convert points and set color based on its label
      pcl::PointXYZRGB p;
      p.x = pc_data[p_i][0];
      p.y = pc_data[p_i][1];
      p.z = pc_data[p_i][2];
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

    // Delete objects
    for (int i = 0; i < number_of_points; i++) delete[] pc_data[i];
    delete[] pc_data;

    // Convert PCL to ROS-msg
    sensor_msgs::msg::PointCloud2 points_msg;
    pcl::toROSMsg(points, points_msg);
    points_msg.header.frame_id = "velodyne";
    points_msg.header.stamp = stamp;

    // Write down to rosbag file
    writer->write<sensor_msgs::msg::PointCloud2>(points_msg, "points", stamp);
  }

  // Close rosbag file
  writer->close();

  std::cout << "finished: bagfile is " << bagfile_path << std::endl;

  return 0;
}

