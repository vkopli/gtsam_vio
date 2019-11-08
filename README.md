# State estimation for legged robots using GTSAM

## Preliminaries
* This package uses the feature tracker and image processor nodelet included with [msckf_vio] (https://github.com/KumarRobotics/msckf_vio).

## Instructions to Run 

using "Kalibr_minotaur_zed.bag" ZED bagfile (collected):
-------------------------------------------------------
run launch file for image processor using ZED camera topic names
- roslaunch legged_vio image_processor_minotaur_zed.launch
-------------------------------------------------------
run ISAM2 for combined CAMERA VIO and ZED CAMERA POSE
- rosrun legged_vio isam2_vio_zedpose (TODO: need to add BetweenFactor)
run ISAM2 for CAMERA VIO alone
- rosrun legged_vio isam2_vio
NOTE: isam2_imu and isam2_vio_imu not working well due to IMU readings not being precise (TODO: update topic names)
-------------------------------------------------------
uncompress ZED image data
rosrun image_transport republish compressed in:=/zed/zed_node/left/image_rect_color raw out:=/zed/zed_node/left/image_rect_color
rosrun image_transport republish compressed in:=/zed/zed_node/right/image_rect_color raw out:=/zed/zed_node/right/image_rect_color
-------------------------------------------------------
visualize estimated pose and 3D locations of features in world frame (isam2_vio/isam2_vio_imu)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_tf_features_config.rviz 
visualize just estimated 3D locations of features in camera frame (isam2_vio/isam2_vio_imu)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_features_camera_config.rviz 
visualize just estimated pose in world frame (isam2_imu)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_tf_config.rviz 
-------------------------------------------------------
play bagfile of data collected from ZED camera for IMU and image information
- rosbag play "path-to-bagfile"
