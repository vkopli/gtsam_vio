# State estimation for legged robots using GTSAM

## Preliminaries
* This package uses the feature tracker and image processor nodelet included with [msckf_vio] (https://github.com/KumarRobotics/msckf_vio).

## Instructions to Change Parameters
-------------------------------------------------------
To change which isam2 node is being run:
change "isam2_node" variable in CMakeLists.txt to one of the below (e.g. "isam2_vio")
- isam2_vio_zedpose - run ISAM2 for combined CAMERA VIO and ZED CAMERA POSE 
- isam2_vio - run ISAM2 for CAMERA VIO alone
- isam2_vio_imu - run ISAM2 for combined CAMERA VIO and ZED IMU alone (bad performance)
- isam2_imu - run ISAM2 for ZED IMU alone (bad performance)
-------------------------------------------------------
To run using raw images:
- change "images_compressed" argument to "false" in "isam2_minotaur_zed.launch"
-------------------------------------------------------

## Instructions to Run 
-------------------------------------------------------
Should have the following topics:
- /zed/zed_node/left/image_rect_color/compressed
- /zed/zed_node/right/image_rect_color/compressed
- /zed/zed_node/odom
- /zed/zed_node/imu/data
-------------------------------------------------------
Might be helpful to have:
- /zed/zed_node/left/camera_info
- /zed/zed_node/right/camera_info
-------------------------------------------------------
Run launch file for image processor using ZED camera topic names:
- roslaunch legged_vio isam2_minotaur_zed.launch
-------------------------------------------------------
Play bagfile of data collected from ZED camera for IMU and image information:
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
