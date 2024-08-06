semantic_kitti_tools
====

## What's this.
ROS 2 Utility for [Semantic KITTI Dataset](https://www.semantic-kitti.org/)

## ground_truth_map
* Make pointcloud map based on velodyne and pose data(provided by official, and made by SuMa)
* Only points with static-object labels are used.
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

### conversion to ROS1bag
By using `rosbags-convert`, resultant ROS2bag is converted to ROS1 bag.

```
$ rosbags-convert --src /tmp/kitti_odometry_bag_with_gt_pose --dst /tmp/kitti_odometry_bag_with_gt_pose.bag --src-typestore ros2_humble
```

#### note
* ROS1 bag file seems not to permit zero-timestamp for messages.
* Static TF's timestamp is set to 1 nsec, if it is zero.
