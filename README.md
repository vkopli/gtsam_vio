# State estimation for legged robots using GTSAM

## Preliminaries
* This package uses the feature tracker and image processor nodelet included with [msckf_vio](https://github.com/KumarRobotics/msckf_vio).

## Instructions to Run 

using ZED bagfile (collected):
-------------------------------------------------------
run launch file for image processor using ZED camera topic names
- roslaunch legged_vio image_processor_zed.launch
-------------------------------------------------------
run ISAM2 for IMU and CAMERA VIO together
- rosrun legged_vio isam2
run ISAM2 for IMU alone
- rosrun legged_vio isam2_imu 
run ISAM2 for CAMERA VIO alone
- rosrun legged_vio isam2_vio
-------------------------------------------------------
visualize estimated pose and 3D locations of features in world frame (isam2/isam2_vio)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_imu_vio_config.rviz 
visualize estimated IMU pose in world frame (isam2_imu)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_imu_config.rviz 
visualize estimated 3D locations of features in camera frame (isam2_vio)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_vio_camera_frame_config.rviz 
-------------------------------------------------------
play bagfile of data collected from ZED camera for IMU and image information
- rosbag play ~/bagfiles/Kalibr_data.bag 

using Euroc dataset (publicly available):
- roslaunch legged_vio image_processor_euroc.launch
- rosrun legged_vio ...
- rviz ...
- rosbag play ...V101.bag (after downloading Vicon Room 1 01 from https://projects.asl.ethz.ch/datasets/doku.php?id=a)
