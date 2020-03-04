// SUBSCRIBER NODE ("isam2")

// ROS/PACKAGE INCLUDES
/* ************************************************************************* */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h> 
#include <gtsam_vio/CameraMeasurement.h>

#include <sensor_msgs/PointCloud2.h>
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
#include <gtsam/navigation/ImuFactor.h> 
#include <gtsam/navigation/CombinedImuFactor.h> 
#include <gtsam/slam/BetweenFactor.h> 

// ADDITIONAL INCLUDES
/* ************************************************************************* */

#include <set>
#include <vector>
#include <memory>
#include <map>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace message_filters;
using namespace gtsam_vio;
using namespace sensor_msgs;
using namespace gtsam;

// PARAMETERS TO SPECIFY FOR OTHER NODES
/* ************************************************************************* */

// topics and frame names being subscribed from or published to
struct LaunchVariables {
  string feature_topic_id;
  string imu_topic_id; 
  string world_frame_id;
  string robot_frame_id;
  string camera_frame_id;
};

// CALLBACK WRAPPER CLASS
/* ************************************************************************* */

class Callbacks { 

private:
  
  int pose_id = 0;

  // Hold ROS node handle initialized in main
  shared_ptr<ros::NodeHandle> nh_ptr;
  
  // Publishers
  tf::TransformBroadcaster tf_pub;
  ros::Publisher feature_cloud_camera_pub;
  ros::Publisher feature_cloud_world_pub; 

  // Create iSAM2 object
  unique_ptr<ISAM2> isam;

  // Initialize factor graph and values estimates on nodes (continually updated by isam.update()) 
  NonlinearFactorGraph graph;
  Values newNodes;
  Values optimizedNodes;       // current estimate of values
  Pose3 prev_robot_pose;       // current estimate of previous pose 
  Vector3 prev_robot_velocity; 
  imuBias::ConstantBias prev_robot_bias;
      
  // Initialize IMU Variables
  PreintegratedImuMeasurements* imu_preintegrated; // CHANGE BACK TO COMBINED: (Combined<->Imu)
  ros::Time prev_imu_timestamp;
    
  // Initialize VIO Variables
  double f;                     // Camera calibration intrinsics
  double cx;
  double cy;
  double resolution_x;          // Image distortion intrinsics
  double resolution_y;
  Cal3_S2Stereo::shared_ptr K;  // Camera calibration intrinsic matrix
  double Tx;                    // Camera calibration extrinsic: distance from cam0 to cam1
  gtsam::Matrix4 T_cam_imu_mat; // Transform to get to camera IMU frame from camera frame
    
  // Noise models
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas(
    (Vector(6) << Vector3::Constant(0.5),Vector3::Constant(0.1)).finished()
  ); // (roll,pitch,yaw in rad; std on x,y,z in meters)
  noiseModel::Diagonal::shared_ptr velocity_noise = noiseModel::Isotropic::Sigma(3, 0.1); // (dim, sigma in m/s)
  noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Isotropic::Sigma(6, 1e-3); // (dim, sigma)
  noiseModel::Isotropic::shared_ptr pose_landmark_noise = noiseModel::Isotropic::Sigma(3, 1.0); // (dim, sigma in pixels): one pixel in u and v
  noiseModel::Isotropic::shared_ptr landmark_noise = noiseModel::Isotropic::Sigma(3, 0.1);

public:

  LaunchVariables lv;
   
  Callbacks(shared_ptr<ros::NodeHandle> nh_ptr_copy) : nh_ptr(move(nh_ptr_copy)) {

    // load topic and frame names
    nh_ptr->getParam("feature_topic_id", lv.feature_topic_id);
    nh_ptr->getParam("imu_topic_id", lv.imu_topic_id);
    nh_ptr->getParam("camera_frame_id", lv.camera_frame_id);
    nh_ptr->getParam("robot_frame_id", lv.robot_frame_id);
    nh_ptr->getParam("world_frame_id", lv.world_frame_id);

    // initialize PointCloud publisher
    this->feature_cloud_camera_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("isam2_feature_point_cloud_camera", 1000);
    this->feature_cloud_world_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("isam2_feature_point_cloud_world", 1000);

    // YAML intrinsics (pinhole): [fu fv pu pv]
    vector<double> cam0_intrinsics(4);
    nh_ptr->getParam("cam0/intrinsics", cam0_intrinsics); // <- neglect right camera 
    this->f = (cam0_intrinsics[0] + cam0_intrinsics[1]) / 2;
    this->cx = cam0_intrinsics[2];  
    this->cy = cam0_intrinsics[3];
    
    // YAML image resolution parameters (radtan): [k1 k2 r1 r2]
    vector<double> cam0_resolution(2);
    nh_ptr->getParam("cam0/resolution", cam0_resolution); // <- neglect right camera
    this->resolution_x = cam0_resolution[0];
    this->resolution_y = cam0_resolution[1];
    
    // YAML extrinsics (distance between 2 cameras and transform between imu and camera))
    vector<double> T_cam1(16);
    nh_ptr->getParam("cam1/T_cn_cnm1", T_cam1);
    this->Tx = T_cam1[3];
    vector<double> T_cam_imu(16);
    nh_ptr->getParam("cam0/T_cam_imu", T_cam_imu);
    gtsam::Matrix4 T_cam_imu_mat_copy(T_cam_imu.data());
    T_cam_imu_mat = move(T_cam_imu_mat_copy);
    
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
    cout << "transform from camera to imu: " << endl << T_cam_imu_mat << endl;
  }

  void callback(const CameraMeasurementConstPtr& camera_msg, const ImuConstPtr& imu_msg) {
  
    // Get IMU orientation, angular vel, and linear acc 
    geometry_msgs::Quaternion orient = imu_msg->orientation; // fields: x, y, z, w
    geometry_msgs::Vector3 ang_vel = imu_msg->angular_velocity; // fields: x, y, z
    geometry_msgs::Vector3 lin_acc = imu_msg->linear_acceleration; // fields: x, y, z

    // Use ImageProcessor to retrieve subscribed features ids and (u,v) image locations for this pose
    vector<FeatureMeasurement> feature_vector = camera_msg->features; 
    
    // Create object to publish PointCloud estimates of features in this pose
    pcl::PointCloud<pcl::PointXYZ>::Ptr feature_cloud_camera_msg_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr feature_cloud_world_msg_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    
    for (unsigned int i = 0; i < feature_vector.size(); i++) { 
      Point3 world_point = processFeature(
        feature_vector[i], prev_robot_pose, feature_cloud_camera_msg_ptr, feature_cloud_world_msg_ptr
      );
    }
    
    // Publish feature PointCloud messages
    feature_cloud_camera_msg_ptr->header.frame_id = lv.camera_frame_id;
    feature_cloud_camera_msg_ptr->height = 1;
    feature_cloud_camera_msg_ptr->width = feature_cloud_camera_msg_ptr->points.size();
    this->feature_cloud_camera_pub.publish(feature_cloud_camera_msg_ptr); 
    feature_cloud_world_msg_ptr->header.frame_id = lv.world_frame_id;
    feature_cloud_world_msg_ptr->height = 1;
    feature_cloud_world_msg_ptr->width = feature_cloud_world_msg_ptr->points.size();
    this->feature_cloud_world_pub.publish(feature_cloud_world_msg_ptr); 
    
    if (pose_id == 0) {
    
      initializeIMUParameters(imu_msg); // initializes imu_preintegrated
      
      // Add priors (pose, velocity, and bias)
      Rot3 prior_rotation = Rot3::Quaternion(orient.x, orient.y, orient.z, orient.w); // quaternion -> Rot3
      prev_robot_pose = Pose3(prior_rotation, Point3(0,0,0)); // start at origin with IMU's starting rotation
      prev_robot_velocity = Vector3();
      prev_robot_bias = imuBias::ConstantBias();
      newNodes.insert(Symbol('x', 0), prev_robot_pose);
      newNodes.insert(Symbol('v', 0), prev_robot_velocity);
      newNodes.insert(Symbol('b', 0), prev_robot_bias);
      graph.emplace_shared< PriorFactor<Pose3> >(Symbol('x', 0), prev_robot_pose, pose_noise);
      graph.emplace_shared< PriorFactor<Vector3> >(Symbol('v', 0), prev_robot_velocity, velocity_noise);
      graph.emplace_shared< PriorFactor<imuBias::ConstantBias> >(Symbol('b', 0), prev_robot_bias, bias_noise);
      
      // Indicate that all node values seen in pose 0 have been seen for next iteration (landmarks)
      optimizedNodes = newNodes; 
      
    } else {
     
      // Print info about this pose to console
      double dt = (imu_msg->header.stamp - prev_imu_timestamp).toSec();
      ROS_INFO("frame %d, dt = %f ms, %lu total features", pose_id, dt*1000, feature_vector.size());
    
      // Integrate current reading from IMU
      imu_preintegrated->integrateMeasurement(
        Vector3(lin_acc.x, lin_acc.y, lin_acc.z), // measuredAcc
        Vector3(ang_vel.x, ang_vel.y, ang_vel.z), // measuredOmega
        dt                                        // time between measurements
      ); 
      
      // Add factors between previous and current state for current IMU measurement
//      graph.emplace_shared<CombinedImuFactor>(
//        Symbol('x', pose_id - 1), Symbol('v', pose_id - 1),
//        Symbol('x', pose_id    ), Symbol('v', pose_id    ),
//        Symbol('b', pose_id - 1), Symbol('b', pose_id    ),
//        *imu_preintegrated
//      );
      graph.emplace_shared<ImuFactor>(
        Symbol('x', pose_id - 1), Symbol('v', pose_id - 1),
        Symbol('x', pose_id    ), Symbol('v', pose_id    ),
        Symbol('b', pose_id - 1), *imu_preintegrated     
      );
      imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
      graph.emplace_shared< BetweenFactor<imuBias::ConstantBias> >(
        Symbol('b', pose_id - 1), 
        Symbol('b', pose_id    ), 
        zero_bias, 
        bias_noise
      );
      
      // Predict initial estimates for current state 
      NavState prev_optimized_state = NavState(prev_robot_pose, prev_robot_velocity);
      NavState propagated_state = imu_preintegrated->predict(prev_optimized_state, prev_robot_bias);
      newNodes.insert(Symbol('x', pose_id), propagated_state.pose()); 
      newNodes.insert(Symbol('v', pose_id), propagated_state.v()); 
      newNodes.insert(Symbol('b', pose_id), prev_robot_bias); 
    
      // UPDATE ISAM WITH NEW FACTORS AND NODES FROM THIS POSE 
      
      isam->update(graph, newNodes); 
      
//      // Print graph to graphviz dot file (render to PDF using "fdp filname.dot -Tpdf > filename.pdf")
//      if (pose_id == 1) {
//        ofstream os("/home/vkopli/Documents/GRASP/Graphs/VisualISAMActualGraph_1pose_2019-09-18.dot");
//        graph.saveGraph(os, newNodes);
//        isam->saveGraph("/home/vkopli/Documents/GRASP/Graphs/VisualISAMGraph_1pose_2019-09-05.dot"); 
//      }

      // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
      // If accuracy is desired at the expense of time, update(*) can be called additional times
      // to perform multiple optimizer iterations every step.
//      isam->update();

      // Update the node values that have been seen up to this point
      optimizedNodes = isam->calculateEstimate();
//      optimizedNodes.print("Current estimate: ");

      // Clear the objects holding new factors and node values for the next iteration
      graph.resize(0);
      newNodes.clear();
      
      // Reset the IMU preintegration object 
      imu_preintegrated->resetIntegrationAndSetBias(prev_robot_bias); 
          
      // Get optimized nodes for next iteration 
      prev_robot_pose = optimizedNodes.at<Pose3>(Symbol('x', pose_id));
      prev_robot_velocity = optimizedNodes.at<Vector3>(Symbol('v', pose_id));
      prev_robot_bias = optimizedNodes.at<imuBias::ConstantBias>(Symbol('b', pose_id));
    }

    prev_imu_timestamp = imu_msg->header.stamp;
    pose_id++;
    
    publishTf(prev_robot_pose, prev_imu_timestamp);
  }

  void publishTf(Pose3 &robot_pose, ros::Time &imu_timestamp) {
  
    tf::Quaternion q_tf;
    tf::Vector3 t_tf;
    tf::quaternionEigenToTF(robot_pose.rotation().toQuaternion(), q_tf);
    tf::vectorEigenToTF(robot_pose.translation(), t_tf);
    tf::Transform world_to_imu_tf = tf::Transform(q_tf, t_tf);
    tf_pub.sendTransform(tf::StampedTransform(
          world_to_imu_tf, imu_timestamp, lv.world_frame_id, lv.robot_frame_id));
  }

  // Add node for feature if not already there and connect to current pose with a factor
  // Add world coordinate of feature to PointCloud (estimated from previous pose)
  Point3 processFeature(FeatureMeasurement feature, 
                        Pose3 prev_robot_pose,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr feature_cloud_camera_msg_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr feature_cloud_world_msg_ptr) {

    Point3 world_point;

    // Identify feature (may appear in previous/future frames) and mark as "seen"
    int landmark_id = feature.id;
    Symbol landmark = Symbol('l', landmark_id);

    double uL = (feature.u0 + 1) * 0.5 * resolution_x;
    double uR = (feature.u1 + 1) * 0.5 * resolution_x ;
    double v = ((feature.v0 + feature.v1) / 2.0 + 1) * 0.5 * resolution_y;

    double d = uR - uL;
    double x = uL;
    double y = v;
    double W = d / this->Tx;

    // Estimated feature location in camera frame
    double X_camera = (x - cx) / W;
    double Y_camera = (y - cy) / W;
    double Z_camera = this->f / W; 
    Point3 camera_point = Point3(X_camera, Y_camera, Z_camera);
    
    // transform landmark coordinates to world frame
    Pose3 prev_camera_pose = Pose3(T_cam_imu_mat) * prev_robot_pose; 
    world_point = prev_camera_pose.transform_from(camera_point);
    
    // if feature is behind camera, don't add to isam2 graph/feature messages
    if (camera_point[2] < 0) {
      return world_point;
    }
    
    // Add location in camera and world frame to PointCloud
    pcl::PointXYZ pcl_camera_point = pcl::PointXYZ(camera_point.x(), camera_point.y(), camera_point.z());
    feature_cloud_camera_msg_ptr->points.push_back(pcl_camera_point);   
    pcl::PointXYZ pcl_world_point = pcl::PointXYZ(world_point.x(), world_point.y(), world_point.z());
    feature_cloud_world_msg_ptr->points.push_back(pcl_world_point);  

	// Add node value for feature/landmark if it doesn't already exist
	bool new_landmark = !optimizedNodes.exists(Symbol('l', landmark_id));
    if (new_landmark) {
      newNodes.insert(landmark, world_point);
    }
    
    // Add factor from this frame's pose to the feature/landmark
    graph.emplace_shared<
      GenericStereoFactor<Pose3, Point3> >(StereoPoint2(uL, uR, v), 
        pose_landmark_noise, Symbol('x', pose_id), landmark, K);
        
    // Add prior to the landmark as well    
    graph.emplace_shared<PriorFactor<Point3> >(landmark, world_point, landmark_noise);
        
    return world_point;
  } 
  
  void initializeIMUParameters(const ImuConstPtr& imu_msg) { 
    
    // Get (constant) IMU covariance of orientation, angular vel, and linear acc (row major about x, y, z axes)
    boost::array<double, 9> orient_cov = imu_msg->orientation_covariance;
    boost::array<double, 9> ang_vel_cov = imu_msg->angular_velocity_covariance;
    boost::array<double, 9> lin_acc_cov = imu_msg->linear_acceleration_covariance;
    
    // Convert covariances to matrix form (Eigen::Matrix<float, 3, 3>)
    gtsam::Matrix3 orient_cov_mat(orient_cov.data());
    gtsam::Matrix3 ang_vel_cov_mat(ang_vel_cov.data());
    gtsam::Matrix3 lin_acc_cov_mat(lin_acc_cov.data());
    std::cout << "Orientation Covariance Matrix (not used): " << std::endl << orient_cov_mat << std::endl; 
    std::cout << "Angular Velocity Covariance Matrix: " << std::endl << ang_vel_cov_mat << std::endl; 
    std::cout << "Linear Acceleration Covariance Matrix: " << std::endl << lin_acc_cov_mat << std::endl; 
    
    // Assign IMU preintegration parameters 
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p =  PreintegratedCombinedMeasurements::Params::MakeSharedU(); 
    p->n_gravity = gtsam::Vector3(-imu_msg->linear_acceleration.x, -imu_msg->linear_acceleration.y, -imu_msg->linear_acceleration.z);
    p->accelerometerCovariance = lin_acc_cov_mat;
    p->integrationCovariance = Matrix33::Identity(3,3)*1e-8; // (DON'T USE "orient_cov_mat": ALL ZEROS)
    p->gyroscopeCovariance = ang_vel_cov_mat; 
    p->biasAccCovariance = Matrix33::Identity(3,3)*pow(0.004905,2); 
    p->biasOmegaCovariance = Matrix33::Identity(3,3)*pow(0.000001454441043,2); 
    p->biasAccOmegaInt = Matrix::Identity(6,6)*1e-5;
    imu_preintegrated = new PreintegratedImuMeasurements(p, imuBias::ConstantBias()); // CHANGE BACK TO COMBINED: (Combined<->Imu)
  }

};

// MAIN
/* ************************************************************************* */
int main(int argc, char **argv) {

  ros::init(argc, argv, "isam2"); // specify name of node and ROS arguments
  shared_ptr<ros::NodeHandle> nh_ptr = make_shared<ros::NodeHandle>();

  // Instantiate class containing callbacks and necessary variables
  Callbacks callbacks_obj(nh_ptr);
  LaunchVariables lv = callbacks_obj.lv;

  // Subscribe to "features" and "imu" topics simultaneously
  message_filters::Subscriber<CameraMeasurement> feature_sub(*nh_ptr, lv.feature_topic_id, 1); 
  message_filters::Subscriber<Imu> imu_sub(*nh_ptr, lv.imu_topic_id, 1); 
  typedef sync_policies::ApproximateTime<CameraMeasurement, Imu> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10000), feature_sub, imu_sub);
  sync.registerCallback(boost::bind(&Callbacks::callback, &callbacks_obj, _1, _2));

  // Loop, pumping all callbacks (specified in subscriber object)
  ros::spin(); 

  return 0;
}
