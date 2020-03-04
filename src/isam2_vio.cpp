// SUBSCRIBER NODE ("isam2_vio")

// ROS/PACKAGE INCLUDES
/* ************************************************************************* */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <gtsam_vio/CameraMeasurement.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h> 
#include <tf_conversions/tf_eigen.h> 

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

// ISAM2 INCLUDES
/* ************************************************************************* */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/StereoFactor.h>

// ADDITIONAL INCLUDES
/* ************************************************************************* */

#include <set>
#include <vector>
#include <memory>
#include <map>
#include <opencv2/opencv.hpp>

using namespace gtsam_vio;
using namespace gtsam;

// PARAMETERS TO SPECIFY FOR OTHER NODES
/* ************************************************************************* */

// topics and frame names being subscribed from or published to
struct LaunchVariables {
  std::string feature_topic_id;
  std::string odom_topic_id; 
  std::string world_frame_id;
  std::string robot_frame_id;
  std::string camera_frame_id;
};

struct RGBColor {
  int r;
  int g;
  int b;
};

// CALLBACK WRAPPER CLASS
/* ************************************************************************* */

class Callbacks { 

private:
  
  int pose_id = 0;

  // Hold ROS node handle initialized in main
  std::shared_ptr<ros::NodeHandle> nh_ptr;
  
  // Publishers 
  ros::Publisher landmark_cloud_pub; 
  tf::TransformBroadcaster tf_pub;

  // Create iSAM2 object
  std::unique_ptr<ISAM2> isam;

  // Initialize factor graph and values estimates on nodes (continually updated by isam.update()) 
  NonlinearFactorGraph graph;
  Values newNodes;
  Values optimizedNodes;       // current estimate of values
  Pose3 prev_camera_pose;      // current estimate of previous pose
    
  // Initialize VIO Variables
  double f;                    // Camera calibration intrinsics
  double cx;
  double cy;
  double resolution_x;         // Image distortion intrinsics
  double resolution_y;
  Cal3_S2Stereo::shared_ptr K; // Camera calibration intrinsic matrix
  double Tx;                   // Camera calibration extrinsic: distance from cam0 to cam1  
  gtsam::Matrix4 T_cam_imu_mat; // Transform to get from camera frame to camera IMU frame
  
  // Noise models
  noiseModel::Isotropic::shared_ptr prior_landmark_noise = noiseModel::Isotropic::Sigma(3, 0.1);
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw 
  noiseModel::Isotropic::shared_ptr pose_landmark_noise = noiseModel::Isotropic::Sigma(3, 10.0); // one pixel in u and v

public:

  LaunchVariables lv;
 
  Callbacks(std::shared_ptr<ros::NodeHandle> nh_ptr_copy) : nh_ptr(std::move(nh_ptr_copy)) {
 
    // load topic and frame names
    nh_ptr->getParam("feature_topic_id", lv.feature_topic_id);
    nh_ptr->getParam("odom_topic_id", lv.odom_topic_id);
    nh_ptr->getParam("camera_frame_id", lv.camera_frame_id);
 //   nh_ptr->getParam("robot_frame_id", lv.robot_frame_id);
    nh_ptr->getParam("world_frame_id", lv.world_frame_id);
 
    // initialize PointCloud publisher
    this->landmark_cloud_pub = nh_ptr->advertise< 
      pcl::PointCloud<pcl::PointXYZRGB> >("isam2_landmark_point_cloud", 1000);

    // YAML intrinsics (pinhole): [fu fv pu pv]
    std::vector<double> cam0_intrinsics(4);
    nh_ptr->getParam("cam0/intrinsics", cam0_intrinsics); // <- neglect right camera 
    this->f = (cam0_intrinsics[0] + cam0_intrinsics[1]) / 2;
    this->cx = cam0_intrinsics[2];  
    this->cy = cam0_intrinsics[3];
    
    // YAML image resolution parameters (radtan): [k1 k2 r1 r2]
    std::vector<double> cam0_resolution(2);
    nh_ptr->getParam("cam0/resolution", cam0_resolution); // <- neglect right camera
    this->resolution_x =  cam0_resolution[0];
    this->resolution_y =  cam0_resolution[1];
    
    // YAML extrinsics (distance between 2 cameras and transform between imu and camera)
    std::vector<double> T_cam1(16);
    nh_ptr->getParam("cam1/T_cn_cnm1", T_cam1);
    this->Tx = T_cam1[3];
    std::vector<double> T_cam_imu(16);
    nh_ptr->getParam("cam0/T_cam_imu", T_cam_imu);
    gtsam::Matrix4 T_cam_imu_mat_copy(T_cam_imu.data());
    T_cam_imu_mat = std::move(T_cam_imu_mat_copy);
    
    // Set K: (fx, fy, s, u0, v0, b) (b: baseline where Z = f*d/b; Tx is negative) 
    this->K.reset(new Cal3_S2Stereo(cam0_intrinsics[0], cam0_intrinsics[1], 0.0, 
      this->cx, this->cy, -this->Tx));
    
    // iSAM2 settings
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam.reset(new ISAM2(parameters));

    // Print to confirm reading the YAML file correctly
    ROS_INFO("cam1/T_cn_cnm1 exists? %d", nh_ptr->hasParam("cam1/T_cn_cnm1"));
    ROS_INFO("Tx: %f", Tx);
    ROS_INFO("cam0/intrinsics exists? %d", nh_ptr->hasParam("cam0/intrinsics")); 
    ROS_INFO("intrinsics: %f, %f, %f, %f", cam0_intrinsics[0], cam0_intrinsics[1], 
      cam0_intrinsics[2], cam0_intrinsics[3]);
    ROS_INFO("cam0/T_cam_imu exists? %d", nh_ptr->hasParam("cam0/T_cam_imu"));
    std::cout << "transform from camera to imu: " << std::endl << T_cam_imu_mat << std::endl;
  }

  void callback(const CameraMeasurementConstPtr& camera_msg, const nav_msgs::OdometryConstPtr& odom_msg) {

    // Add node value for current pose with initial estimate being previous pose
    if (pose_id == 0 || pose_id == 1) {
      prev_camera_pose = Pose3() * Pose3(T_cam_imu_mat);
    } 
    newNodes.insert(Symbol('x', pose_id), prev_camera_pose);

    // Use ImageProcessor to retrieve subscribed features ids and (u,v) image locations for this pose
    std::vector<FeatureMeasurement> feature_vector = camera_msg->features; 
               
    // Print info about this pose to console
    ROS_INFO("frame %d, %lu total features, camera position: (%f, %f, %f)", pose_id, feature_vector.size(), prev_camera_pose.x(), prev_camera_pose.y(), prev_camera_pose.z());
    
    // Create object to publish PointCloud of landmarks
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landmark_cloud_msg_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    // Convert features from image_processor to landmarks with 3D coordinates and add to ISAM2 graph/point cloud
    for (unsigned int i = 0; i < feature_vector.size(); i++) { 
      featureToLandmark(feature_vector[i], prev_camera_pose, landmark_cloud_msg_ptr);
    }
    
    // Publish landmark PointCloud message (in world frame)
    landmark_cloud_msg_ptr->header.frame_id = lv.world_frame_id;
    landmark_cloud_msg_ptr->height = 1;
    landmark_cloud_msg_ptr->width = landmark_cloud_msg_ptr->points.size();
    this->landmark_cloud_pub.publish(landmark_cloud_msg_ptr); 
          
    if (pose_id == 0) {

      // Add prior on pose x0 (zero pose is used to set world frame)
      graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), Pose3(), pose_noise);

      // Indicate that all node values seen in pose 0 have been seen for next iteration 
      optimizedNodes = newNodes; 

    } else {
    
      // Update ISAM2 graph with new nodes and factors from this pose, optimize graphs
      isam->update(graph, newNodes); 

      // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
      // If accuracy is desired at the expense of time, update(*) can be called additional times
      // to perform multiple optimizer iterations every step.
//      isam->update();

      // Update the node values that have been seen up to this point
      optimizedNodes = isam->calculateEstimate();
//      optimizedNodes.print("Current estimate: ");

//      // Print graph to graphviz dot file (render to PDF using "fdp filname.dot -Tpdf > filename.pdf")
//      if (pose_id == 1) {
//        ofstream os("/home/vkopli/Documents/GRASP/Graphs/VisualISAMActualGraph_1pose_2019-09-18.dot");
//        graph.saveGraph(os, newNodes);
//        isam->saveGraph("/home/vkopli/Documents/GRASP/Graphs/VisualISAMGraph_1pose_2019-09-05.dot"); 
//      }

      // Clear the objects holding new factors and node values for the next iteration
      graph.resize(0);
      newNodes.clear();
      
      // Get optimized nodes for next iteration 
      prev_camera_pose = optimizedNodes.at<Pose3>(Symbol('x', pose_id));
    }

    ros::Time timestamp = camera_msg->header.stamp;
    pose_id++;
    
    publishTf(prev_camera_pose, timestamp);
  }

  void publishTf(Pose3 &camera_pose, ros::Time &timestamp) {
    
    tf::Quaternion q_tf;
    tf::Vector3 t_tf;
    tf::quaternionEigenToTF(camera_pose.rotation().toQuaternion(), q_tf); 
    tf::vectorEigenToTF(camera_pose.translation(), t_tf);
    tf::Transform world_to_imu_tf = tf::Transform(q_tf, t_tf);
    tf_pub.sendTransform(tf::StampedTransform(
      world_to_imu_tf, timestamp, lv.world_frame_id, lv.camera_frame_id)); 
  }

  // Transform feature from image_processor to landmark with 3D coordinates
  // Add landmark to ISAM2 graph if not already there (connect to current pose with a factor)
  // Add landmark to point cloud 
  void featureToLandmark(FeatureMeasurement feature, 
                           Pose3 prev_camera_pose,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr landmark_cloud_msg_ptr) {

    // Identify feature/landmark (may appear in previous/future frames) and mark as "seen"
    int landmark_id = feature.id;
    Symbol landmark = Symbol('l', landmark_id);

    double uL = (feature.u0 + 1) * 0.5 * resolution_x;
    double uR = (feature.u1 + 1) * 0.5 * resolution_x ;
    double v = ((feature.v0 + feature.v1) / 2.0 + 1) * 0.5 * resolution_y;

    double d = uR - uL;
    double x = uL;
    double y = v;
    double W = d / this->Tx;

    // Estimate feature location in camera frame
    double X_camera = (x - cx) / W;
    double Y_camera = (y - cy) / W;
    double Z_camera = this->f / W; 
    Point3 camera_point = Point3(X_camera, Y_camera, Z_camera);
        
    // If landmark is behind camera, don't add to isam2 graph/point cloud
    if (camera_point[2] < 0) {
      return;
    }
    
    // Transform landmark coordinates to world frame 
    Point3 world_point = prev_camera_pose.transform_from(camera_point); 
    
    // Add landmark to point cloud (in world frame)
    RGBColor rgb = getLandmarkColor(landmark_id);
    pcl::PointXYZRGB pcl_world_point = pcl::PointXYZRGB(rgb.r, rgb.g, rgb.b);
    pcl_world_point.x = world_point.x();
    pcl_world_point.y = world_point.y();
    pcl_world_point.z = world_point.z(); 
    landmark_cloud_msg_ptr->points.push_back(pcl_world_point);  

	  // Add ISAM2 value for feature/landmark if it doesn't already exist
	  bool bool_new_landmark = !optimizedNodes.exists(Symbol('l', landmark_id));
    if (bool_new_landmark) {
      newNodes.insert(landmark, world_point);
    }
    
    // Add ISAM2 factor connecting this frame's pose to the landmark
    graph.emplace_shared<
      GenericStereoFactor<Pose3, Point3> >(StereoPoint2(uL, uR, v), 
        pose_landmark_noise, Symbol('x', pose_id), landmark, K);
        
    // Removing this causes greater accuracy but earlier gtsam::IndeterminantLinearSystemException)
    // Add prior to the landmark as well    
    graph.emplace_shared<PriorFactor<Point3> >(landmark, world_point, prior_landmark_noise);
  } 
  
  RGBColor getLandmarkColor(int id) {
    double center = 128;
    double width = 127;
    double r_freq = 1.666;
    double g_freq = 2.666;
    double b_freq = 3.666;
    RGBColor rgb;
    rgb.r = std::sin(r_freq * (double) id) * width + center;
    rgb.g = std::sin(g_freq * (double) id) * width + center;
    rgb.b = std::sin(b_freq * (double) id) * width + center;
    return rgb;
  }

};

// MAIN
/* ************************************************************************* */
int main(int argc, char **argv) {

  ros::init(argc, argv, "isam2"); // specify name of node and ROS arguments
  std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>();

  // Instantiate class containing callbacks and necessary variables
  Callbacks callbacks_obj(nh_ptr);
  LaunchVariables lv = callbacks_obj.lv;

  // Subscribe to "features" and "ZED odom" topics simultaneously
  message_filters::Subscriber<CameraMeasurement> feature_sub(*nh_ptr, lv.feature_topic_id, 1); 
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(*nh_ptr, lv.odom_topic_id, 1); 
  typedef message_filters::sync_policies::ApproximateTime<CameraMeasurement, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10000), feature_sub, odom_sub);
  sync.registerCallback(boost::bind(&Callbacks::callback, &callbacks_obj, _1, _2));

  // Loop, pumping all callbacks (specified in subscriber object)
  ros::spin(); 

  return 0;
}
