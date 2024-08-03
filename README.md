semantic_kitti_tools
====

my utility

## ground_truth_map
* Make pointcloud map based on velodyne and pose data(provided by official, and made by SuMa)
* Save the map into `/tmp/kitti_map.pcd`

### usage
```
$ ros2 run semantic_kitti_tools ground_truth_map <target_sequence_dir> <calibration_dir>
$ ros2 run semantic_kitti_tools ground_truth_map ~/semantic_kitti/05  ~/data_odometry_calib/dataset/sequence/05 # example
```

## make_rosbag2
* Make bag file for ROS2
* If \<is_set_groundtruth_pose\> is set to true, TF from `/world` to `/base_link` is written to bagfile.
* Currently, only labeled pointcloud (and TF) is written, the image is not written.

### usage
```
$ ros2 run semantic_kitti_tools make_rosbag2 <target_sequence_dir> <calibration_dir> <is_set_groundtruth_pose>
$ ros2 run semantic_kitti_tools make_rosbag2 ~/semantic_kitti/05  ~/data_odometry_calib/dataset/sequence/05 true # example
```
