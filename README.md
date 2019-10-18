# State estimation for legged robots using GTSAM

## Preliminaries
* This package uses the feature tracker and image processor nodelet included with [msckf_vio](https://github.com/KumarRobotics/msckf_vio).

## Instructions to Run 

using Euroc dataset (publicly available):
- roslaunch legged_vio image_processor_euroc.launch
- rosrun legged_vio isam2 (isam2_vio if running on just camera, isam2_imu if running on just imu)
- rviz ~/catkin_ws/src/legged_vio/rviz/rviz_zed_config.rviz (visualize isam2/isam2_vio)
- rosbag play ~/bagfiles/Kalibr_data.bag (after downloading Vicon Room 1 01 from https://projects.asl.ethz.ch/datasets/doku.php?id=a)

using ZED dataset (Kalibr data, not publicly available):
- roslaunch legged_vio image_processor_zed.launch
- rosrun legged_vio isam2
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_tf_zed_config.rviz (visualize estimated 3D locations of features in world frame, works for isam2.cpp)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_zed_config.rviz (visualize estimated 3D locations of features in camera frame, works for isam2.cpp and isam2_vio.cpp)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_tf_config.rviz (visualize estimated IMU pose in world frame, works for isam2.cpp and isam2_imu.cpp)
- rosbag play ~/bagfiles/Kalibr_data.bag (requires the bagfile to be downloaded)
