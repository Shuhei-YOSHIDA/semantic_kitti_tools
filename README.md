semantic_kitti_tools
====

my utility

## ground_truth_map
* Make pointcloud map based on velodyne and pose data(provided by official, and made by SuMa)

### usage
```
$ ros2 run semantic_kitti_tools ground_truth_map <target_sequence_dir> <calibration_dir>
$ ros2 run semantic_kitti_tools ground_truth_map ~/semantic_kitti/05  ~/data_odometry_calib/dataset/sequence/05 # example
```
