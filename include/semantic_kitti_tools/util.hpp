/**
 * @file util.hpp
 */

#ifndef INCLUDE_SEMANTIC_KITTI_TOOLS_UTIL_H
#define INCLUDE_SEMANTIC_KITTI_TOOLS_UTIL_H

#include <cstdint>
#include <map>
#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <Eigen/Dense>

namespace semantic_kitti_tools
{

void loadPointCloudFromFile(
    const std::string& file_path,
    float **&pc_data, // to be deleted after usage
    long &number_of_points);

void loadPointLabelFromFile(
    const std::string& file_path,
    std::vector<uint16_t>& labels,
    std::vector<uint16_t>& instance_id,
    long &number_of_labels);

void loadPoseFromFile(
    const std::string& textfile_path,
    std::vector<Eigen::Vector3d>& pos_list,
    std::vector<Eigen::Matrix3d>& rot_list,
    long& line_count);

void LoadLiDARCalibFromFile(
    const std::string& textfile_path,
    Eigen::Matrix4d& tr);

} // namespace semantic_kitti_tool


#endif /* ifndef INCLUDE_SEMANTIC_KITTI_TOOLS_UTIL_H */
