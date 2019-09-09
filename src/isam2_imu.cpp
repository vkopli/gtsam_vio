// SUBSCRIBER NODE ("isam2_imu") <-- TODO: integrate extra lines marked by // ** into isam2.cpp

// ROS/PACKAGE INCLUDES
/* ************************************************************************* */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <legged_vio/CameraMeasurement.h>
#include <sensor_msgs/Imu.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

// ISAM2 INCLUDES
/* ************************************************************************* */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/navigation/CombinedImuFactor.h>

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
#include <gtsam/slam/ProjectionFactor.h>

// ADDITIONAL INCLUDES
/* ************************************************************************* */

#include <set>
#include <vector>
#include <memory>
#include <map>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace message_filters;
using namespace legged_vio;
using namespace sensor_msgs;
using namespace gtsam;

// PARAMETERS TO SPECIFY FOR OTHER NODES
/* ************************************************************************* */

struct LaunchVariables {
  string feature_topic_id = "minitaur/image_processor/features";
  string imu_topic_id = "/zed/imu/data_raw"; // "/zed/imu/data_raw"; // "/imu0"
  string fixed_frame_id = "zed_left_camera_optical_frame"; // "zed_left_camera_optical_frame"; // "map"
};

// CALLBACK WRAPPER CLASS
/* ************************************************************************* */

class Callbacks { 

private:

  LaunchVariables lv;
  
  int pose_id = 0;

  // Hold ROS node handle initialized in main
  shared_ptr<ros::NodeHandle> nh_ptr;

  // Create iSAM2 object
  unique_ptr<ISAM2> isam;

  // Initialize Factor Graph and Values Estimates on Nodes (continually updated by isam.update()) 
  NonlinearFactorGraph graph;
  Values newNodes;
  Values optimizedNodes; // current estimate of values
  
  // Initialize VIO Variables
  Pose3 prevOptimizedPose; // current estimate of previous pose
  
  // Initialize IMU Variables // **
  Vector3 prevOptimizedVelocity;
  imuBias::ConstantBias prevOptimizedBias;
  PreintegratedCombinedMeasurements* preint_imu_combined;
  NavState prev_state; // previous state for the imu integration
  NavState prop_state; // current estimate of state
  imuBias::ConstantBias prev_bias;
  
  // Noise models // ** (pose_noise is a duplicate variable)
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Isotropic::Sigma(6,1e-3);
  

public:
 
  Callbacks(shared_ptr<ros::NodeHandle> nh_ptr_copy) : nh_ptr(move(nh_ptr_copy)) {

    // iSAM2 settings
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam.reset(new ISAM2(parameters));

  }

  void callback(const CameraMeasurementConstPtr& camera_msg, const ImuConstPtr& imu_msg) {
          
    // Set up IMU priors // **
    if (pose_id == 0) {
    
      // Get (constant) IMU covariance of orientation, angular vel, and linear acc (row major about x, y, z axes)
      boost::array<double, 9> orient_cov = imu_msg->orientation_covariance;
      boost::array<double, 9> ang_vel_cov = imu_msg->angular_velocity_covariance;
      boost::array<double, 9> lin_acc_cov = imu_msg->linear_acceleration_covariance;
      
      // Convert covariances to matrix form (Eigen::Matrix<float, 3, 3>)
      gtsam::Matrix3 orient_cov_mat;
      gtsam::Matrix3 ang_vel_cov_mat;
      gtsam::Matrix3 lin_acc_cov_mat;
      orient_cov_mat << orient_cov[0], orient_cov[1], orient_cov[2], orient_cov[3], orient_cov[4], 
                        orient_cov[5],orient_cov[6], orient_cov[7], orient_cov[8];
      ang_vel_cov_mat << ang_vel_cov[0], ang_vel_cov[1], ang_vel_cov[2], ang_vel_cov[3], ang_vel_cov[4], 
                         ang_vel_cov[5],ang_vel_cov[6], ang_vel_cov[7], ang_vel_cov[8];
      lin_acc_cov_mat << lin_acc_cov[0], lin_acc_cov[1], lin_acc_cov[2], lin_acc_cov[3], lin_acc_cov[4], 
                         lin_acc_cov[5],lin_acc_cov[6], lin_acc_cov[7], lin_acc_cov[8];   
      std::cout << "Orientation Covariance Matrix: " << std::endl << orient_cov_mat << std::endl; 
      std::cout << "Angular Velocity Covariance Matrix: " << std::endl << ang_vel_cov_mat << std::endl; 
      std::cout << "Linear Acceleration Covariance Matrix: " << std::endl << lin_acc_cov_mat << std::endl; 
      
      // Assign IMU priors
      boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
      // PreintegrationBase params:
      p->accelerometerCovariance = lin_acc_cov_mat; // acc white noise in continuous
      p->integrationCovariance = orient_cov_mat; // integration uncertainty continuous
      // should be using 2nd order integration
      // PreintegratedRotation params:
      p->gyroscopeCovariance = ang_vel_cov_mat; // gyro white noise in continuous
      // PreintegrationCombinedMeasurements params:
      p->biasAccCovariance = Matrix33::Identity(3,3) *  1e-5; // acc bias in continuous
      p->biasOmegaCovariance = Matrix33::Identity(3,3) * 1e-5;; // gyro bias in continuous
      p->biasAccOmegaInt = Matrix::Identity(6,6) * 1e-5; // error in the bias used for preintegration
      
      // Set IMU Variables
      preint_imu_combined = new PreintegratedCombinedMeasurements(p, imuBias::ConstantBias());
      prev_state = NavState(Pose3(), Vector3()); // assume 0 prior pose and velocity
      prop_state = prev_state; 
      prev_bias = imuBias::ConstantBias(); // assume 0 initial bias 
      
    }
              
    // Add node value for current pose based on previous pose // ** (prevOptimizedPose is duplicate variable)
    if (pose_id == 0 || pose_id == 1) {
      prevOptimizedPose = Pose3();
      prevOptimizedVelocity = Vector3();
      prevOptimizedBias = imuBias::ConstantBias();
    } else {
      prevOptimizedPose = optimizedNodes.at<Pose3>(Symbol('x', pose_id - 1));
      prevOptimizedVelocity = optimizedNodes.at<Vector3>(Symbol('v', pose_id - 1));
      prevOptimizedBias = optimizedNodes.at<imuBias::ConstantBias>(Symbol('b', pose_id - 1));
    } 
    newNodes.insert(Symbol('x', pose_id), prevOptimizedPose);
    newNodes.insert(Symbol('v', pose_id), prevOptimizedVelocity);
    newNodes.insert(Symbol('b', pose_id), prevOptimizedBias);
    
    // Add IMU node with initial estimate for current pose based on previous pose
//    newNodes.insert(Symbol('x', pose_id), prior_pose);
//    newNodes.insert(Symbol('v', pose_id), prior_velocity);
//    newNodes.insert(Symbol('b', pose_id), prior_imu_bias); 
            
    // Get IMU orientation, angular vel, and linear acc // **
    geometry_msgs::Quaternion orient = imu_msg->orientation; // fields: x, y, z, w
    geometry_msgs::Vector3 ang_vel = imu_msg->angular_velocity; // fields: x, y, z
    geometry_msgs::Vector3 lin_acc = imu_msg->linear_acceleration; // fields: x, y, z
    
    ROS_INFO("orientation: %f, %f, %f, %f", orient.x, orient.y, orient.z, orient.w);
    ROS_INFO("angular velocity: %f, %f, %f", ang_vel.x, ang_vel.y, ang_vel.z);
    ROS_INFO("linear acceleration: %f, %f, %f", lin_acc.x, lin_acc.y, lin_acc.z);

    if (pose_id == 0) {

      // Add prior on pose x0 (zero pose is used to set world frame)
      graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), Pose3(), pose_noise);

      // Indicate that all node values seen in pose 0 have been seen for next iteration 
      optimizedNodes = newNodes; 

    } else {
    
      // Update iSAM with new factors and node values from this pose
      
//      ROS_INFO("before update step");
      isam->update(graph, newNodes); 

      // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
      // If accuracy is desired at the expense of time, update(*) can be called additional times
      // to perform multiple optimizer iterations every step.
//      isam->update();
//      ROS_INFO("after update step");

      // Update the node values that have been seen up to this point
      optimizedNodes = isam->calculateEstimate();
//      optimizedNodes.print("Current estimate: ");

      // Clear the objects holding new factors and node values for the next iteration
      graph.resize(0);
      newNodes.clear();
    }

    pose_id++;
  }

};

// MAIN (same as isam2_imu.cpp)
/* ************************************************************************* */
int main(int argc, char **argv) {

  LaunchVariables lv;

  ros::init(argc, argv, "isam2"); // specify name of node and ROS arguments
  shared_ptr<ros::NodeHandle> nh_ptr = make_shared<ros::NodeHandle>();

  // Instantiate class containing callbacks and necessary variables
  Callbacks callbacks_obj(nh_ptr);

  // Subscribe to "features" and "imu" topics simultaneously
  message_filters::Subscriber<CameraMeasurement> feature_sub(*nh_ptr, lv.feature_topic_id, 1); 
  message_filters::Subscriber<Imu> imu_sub(*nh_ptr, lv.imu_topic_id, 1); 
  TimeSynchronizer<CameraMeasurement, Imu> sync(feature_sub, imu_sub, 10);
  sync.registerCallback(boost::bind(&Callbacks::callback, &callbacks_obj, _1, _2));

  // Loop, pumping all callbacks (specified in subscriber object)
  ros::spin(); 

  return 0;
}
