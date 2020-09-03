# SLAM for ZED Stereo Camera using iSAM2

## Summary

Presentation on (1) theoretical background of iSAM2 and (2) results on Turtlebot (videos at the end):
- https://www.dropbox.com/s/m61mxbnu8e43lp5/ISAM2%20Presentation.pptx?dl=0

Paper: https://www.dropbox.com/s/dka68k9i4uw187r/Master_s_Thesis.pdf?dl=0

Dataset: https://www.dropbox.com/sh/vku3rpquwpql0h0/AADmsJg6yzNQ7nIK3XmbF7iva?dl=0

## Preliminaries

This package uses: 
* the iSAM2 and factor graph tools from [gtsam] (https://github.com/borglab/gtsam).
* the feature tracker and image processor nodelet included with [msckf_vio] (https://github.com/KumarRobotics/msckf_vio).

When building gtsam from source, use the following cmake flags: -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_WRAP=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_TYPEDEF_POINTS_TO_VECTORS=ON

## Instructions to Run 

Run launch file:
- roslaunch gtsam_vio isam2_turtlebot_zed.launch

Run bag file from dataset link above (or use your own ZED mini stereo camera):
- rosbag play turtlebot_zed_forward_back.bag

Make sure the following topics are publishing messages:
- /zed/zed_node/left/image_rect_color/compressed
- /zed/zed_node/right/image_rect_color/compressed
- /zed/zed_node/odom
- /zed/zed_node/imu/data (only necessary for isam2_vio_imu & isam2_imu implementations)

## Instructions to Visualize

To visualize the estimated camera pose and 3D locations of features in the world frame, run the following command:
- rviz rviz -d ~/catkin_ws/src/gtsam_vio/rviz/rviz_tf_features_config.rviz 

## Instructions to Change Launch Specifications

To change which iSAM2 implementation is being run:
change the "isam2_node" definition in CMakeLists.txt to one of the below (e.g. "isam2_vio_zedpose")
- isam2_vio_zedpose - run iSAM2 for combined CAMERA VIO and ZED ODOMETRY OUTPUT 
- isam2_vio - run iSAM2 for CAMERA VIO alone
- isam2_vio_imu - run iSAM2 for combined CAMERA VIO and RAW ZED IMU OUTPUT (bad performance)
- isam2_imu - run iSAM2 for RAW ZED IMU OUTPUT alone (bad performance)

To change frame and camera topic specifications:
- change "iSAM2 Variables" and "iSAM2 Parameters" at the top of isam2_turtlebot_zed.launch
