# State estimation for legged robots using GTSAM

## Preliminaries
* This package uses the feature tracker and image processor nodelet included with [msckf_vio] (https://github.com/KumarRobotics/msckf_vio).

## Instructions to Change Parameters
-------------------------------------------------------
To change which isam2 node is being run:
- isam2_vio_zedpose - run ISAM2 for combined CAMERA VIO and ZED CAMERA POSE (TODO: need to add BetweenFactor)
- isam2_vio - run ISAM2 for CAMERA VIO alone
- isam2_vio_imu - run ISAM2 for combined CAMERA VIO and ZED IMU alone (bad performance)
- isam2_imu - run ISAM2 for ZED IMU alone (bad performance)
change "isam2_node" argument to node name (e.g. "isam2_vio") in "isam2_minotaur_zed.launch"
-------------------------------------------------------
To run using raw images:
- change "images_compressed" argument to "false" in "isam2_minotaur_zed.launch"
-------------------------------------------------------

## Instructions to Run 

USING "Kalibr_minotaur_zed.bag" ZED BAGFILE (COLLECTED):
-------------------------------------------------------
run launch file for image processor using ZED camera topic names
- roslaunch legged_vio isam2_minotaur_zed.launch
-------------------------------------------------------
play bagfile of data collected from ZED camera for IMU and image information
- rosbag play "path-to-bagfile"
-------------------------------------------------------

## Instructions to Visualize
-------------------------------------------------------
visualize estimated pose and 3D locations of features in world frame (isam2_vio/isam2_vio_imu)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_tf_features_config.rviz 
-------------------------------------------------------
visualize just estimated 3D locations of features in camera frame (isam2_vio/isam2_vio_imu)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_features_camera_config.rviz
------------------------------------------------------- 
visualize just estimated pose in world frame (isam2_imu)
- rviz rviz -d ~/catkin_ws/src/legged_vio/rviz/rviz_tf_config.rviz 
-------------------------------------------------------
