/**
 * @file util.cpp
 */

#include "semantic_kitti_tools/util.hpp"
//#include "semantic_kitti_tools/param.hpp"

namespace semantic_kitti_tools
{

void loadPointCloudFromFile(
    const std::string& file_path,
    float **&pc_data, // to be deleted after usage
    long &number_of_points)
{
  std::FILE *p_file = fopen(file_path.c_str(), "rb");
  if (p_file==NULL)
  {
    std::cerr << "pointcloud file open error" << std::endl;
    exit(-1);
  }
  fseek(p_file, 0, SEEK_END); // file pointer goes to the end of the file
  long file_size = ftell(p_file); // file size which is represented by byte unit
  rewind(p_file); // rewind file pointer to the beginning
  float *raw_data = new float[file_size]; // over-estimated? float[file_size/sizeof(float)]?
  fread(raw_data, sizeof(float), file_size/sizeof(float), p_file);
  fclose(p_file);
  number_of_points = file_size / 4 / sizeof(float); // each point field is x,y,z,r
  pc_data = new float*[number_of_points];
  for (int i = 0; i < number_of_points; i++)
  {
    pc_data[i] = new float[4];
    for (int j = 0; j < 4; j++)
    {
      pc_data[i][j] = raw_data[4*i+j];
    }
  }
  delete[] raw_data;
}

void loadPointLabelFromFile(
    const std::string& file_path,
    std::vector<uint16_t>& labels,
    std::vector<uint16_t>& instance_id,
    long &number_of_labels)
{
  std::FILE *p_file = fopen(file_path.c_str(), "rb");
  if (p_file==NULL)
  {
    std::cerr << "label file open error" << std::endl;
    exit(-1);
  }
  fseek(p_file, 0, SEEK_END);
  long file_size = ftell(p_file);
  rewind(p_file);
  // lower 16bits: label, upper 16bits: instance id
  uint32_t *raw_data = new uint32_t[file_size];
  fread(raw_data, sizeof(uint32_t), file_size/sizeof(uint32_t), p_file);
  fclose(p_file);
  number_of_labels = file_size/sizeof(uint32_t);

  labels.clear();
  instance_id.clear();
  labels.reserve(number_of_labels);
  instance_id.reserve(number_of_labels);

  for (int i = 0; i < number_of_labels; i++)
  {
    uint32_t d = raw_data[i];
    // upper 16bits
    uint16_t upper = (d >> 16);
    // lower 16bits
    uint16_t lower = (0x0000ffff & d);

    labels.push_back(lower);
    instance_id.push_back(upper);
  }

  delete[] raw_data;
}

void loadPoseFromFile(
    const std::string& textfile_path,
    std::vector<Eigen::Vector3d>& pos_list,
    std::vector<Eigen::Matrix3d>& rot_list,
    long& line_count)
{
  std::ifstream pose_file(textfile_path);

  if (!pose_file)
  {
    std::cerr << "Failed to open the file" << std::endl;
    exit(-1);
  }

  line_count = 0;
  std::string line;

  while (std::getline(pose_file, line)) 
  {
    line_count++;
  }

  pose_file.clear();
  pose_file.seekg(0);

  pos_list.clear();
  pos_list.reserve(line_count);
  rot_list.clear();
  rot_list.reserve(line_count);

  while (std::getline(pose_file, line))
  {
    std::stringstream ss(line);
    std::string s;
    std::vector<double> v;
    v.reserve(12);
    while (getline(ss, s, ' '))
    {
      double vi = std::stod(s);
      v.push_back(vi);
    }

    Eigen::Vector3d p(v[3], v[7], v[11]);
    Eigen::Matrix3d r;
    r << v[0], v[1], v[2],
         v[4], v[5], v[6],
         v[8], v[9], v[10];
    pos_list.push_back(p);
    rot_list.push_back(r);
  }
}

void LoadLiDARCalibFromFile(
    const std::string& textfile_path,
    Eigen::Matrix4d& tr)
{
  // Load text file and read a line which start at "Tr:".
  // Calibration data is 4x3 matrix elements.
  std::ifstream calib_file(textfile_path);

  if (!calib_file)
  {
    std::cerr << "Failed to open the file" << std::endl;
    exit(-1);
  }

  std::string line;
  bool found = false;
  while (std::getline(calib_file, line))
  {
    if (line.substr(0, 3) == "Tr:")
    {
      found = true;
      break;
    }
  }

  if (!found)
  {
    std::cerr << "Not found calibration data" << std::endl;
    exit(-1);
  }

  std::stringstream ss(line);
  std::string s;
  std::array<double, 12> a;

  int idx = 0;
  while (getline(ss, s, ' '))
  {
    if (s == "Tr:") continue;
    double ai = std::stod(s);
    a[idx] = ai;
    idx++;
  }

  tr << a[0], a[1], a[2], a[3],
        a[4], a[5], a[6], a[7],
        a[8], a[9], a[10], a[11],
        0, 0, 0, 1;
}



} // namespace semantic_kitti_tools
