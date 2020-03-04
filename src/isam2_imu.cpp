// SUBSCRIBER NODE ("isam2_imu") <-- lines marked by "// **" were added into isam2_vio to make isam2

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
#include <gtsam/navigation/ImuFactor.h> // **
#include <gtsam/navigation/CombinedImuFactor.h> // **
#include <gtsam/slam/BetweenFactor.h> // **

// ADDITIONAL INCLUDES
/* ************************************************************************* */

#include <set>
#include <vector>
#include <memory>
#include <map>
#include <opencv2/opencv.hpp>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

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

  // Create iSAM2 object
  unique_ptr<ISAM2> isam;

  // Initialize Factor Graph and Values Estimates on Nodes (continually updated by isam.update()) 
  NonlinearFactorGraph graph;
  Values newNodes;
  Values optimizedNodes; // current estimate of values
  Pose3 prev_robot_pose; // current estimate of previous pose
  Vector3 prev_robot_velocity; // **
  imuBias::ConstantBias prev_robot_bias; // **
  
  // Initialize IMU Variables // **
  PreintegratedCombinedMeasurements* imu_preintegrated; // CHANGE BACK TO COMBINED (Combined<->Imu)
  ros::Time prev_imu_timestamp;
    
  // Noise models // ** (pose_noise is a duplicate variable)
  noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas(
    (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished() // (roll,pitch,yaw in rad; std on x,y,z in meters)
  );
  noiseModel::Diagonal::shared_ptr velocity_noise = noiseModel::Isotropic::Sigma(3,0.1); // (dim, sigma in m/s)
  noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Isotropic::Sigma(6,1e-3);

public:

  LaunchVariables lv;
 
  Callbacks(shared_ptr<ros::NodeHandle> nh_ptr_copy) : nh_ptr(move(nh_ptr_copy)) {

    // load topic and frame names
    nh_ptr->getParam("feature_topic_id", lv.feature_topic_id);
    nh_ptr->getParam("imu_topic_id", lv.imu_topic_id);
    nh_ptr->getParam("camera_frame_id", lv.camera_frame_id);
    nh_ptr->getParam("robot_frame_id", lv.robot_frame_id);
    nh_ptr->getParam("world_frame_id", lv.world_frame_id);

    // iSAM2 settings
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam.reset(new ISAM2(parameters));

  }

  void callback(const CameraMeasurementConstPtr& camera_msg, const ImuConstPtr& imu_msg) {
            
    // Get IMU orientation, angular vel, and linear acc // **
    geometry_msgs::Quaternion orient = imu_msg->orientation; // fields: x, y, z, w
    geometry_msgs::Vector3 ang_vel = imu_msg->angular_velocity; // fields: x, y, z
    geometry_msgs::Vector3 lin_acc = imu_msg->linear_acceleration; // fields: x, y, z
    
//    ROS_INFO("orientation: %f, %f, %f, %f", orient.x, orient.y, orient.z, orient.w);
//    ROS_INFO("angular velocity: %f, %f, %f", ang_vel.x, ang_vel.y, ang_vel.z);
//    ROS_INFO("linear acceleration: %f, %f, %f", lin_acc.x, lin_acc.y, lin_acc.z);

    if (pose_id == 0) {
    
      initializeIMUParameters(imu_msg); // initializes imu_preintegrated // **

      // Add priors (pose, velocity, and bias) // ** (altered from isam2.cpp)
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

      //** "optimizedNodes = newNodes;" (deleted from isam2_vio.cpp)     

    } else {
      
      // Integrate current reading from IMU // **
      double dt = (imu_msg->header.stamp - prev_imu_timestamp).toSec();
      imu_preintegrated->integrateMeasurement(
        Vector3(lin_acc.x, lin_acc.y, lin_acc.z), // measuredAcc
        Vector3(ang_vel.x, ang_vel.y, ang_vel.z), // measuredOmega
        dt                                        // time between measurements
      ); 
      
      // Add factors between previous and current state for current IMU measurement // **      
      // graph.emplace_shared<ImuFactor>(
      //   Symbol('x', pose_id - 1), Symbol('v', pose_id - 1),
      //   Symbol('x', pose_id    ), Symbol('v', pose_id    ),
      //   Symbol('b', pose_id - 1), *imu_preintegrated     
      // );
      // imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
      // graph.emplace_shared< BetweenFactor<imuBias::ConstantBias> >(
      //   Symbol('b', pose_id - 1), 
      //   Symbol('b', pose_id    ), 
      //   zero_bias, 
      //   bias_noise
      // );
     graph.emplace_shared<CombinedImuFactor>(
       Symbol('x', pose_id - 1), Symbol('v', pose_id - 1),
       Symbol('x', pose_id    ), Symbol('v', pose_id    ),
       Symbol('b', pose_id - 1), Symbol('b', pose_id    ),
       *imu_preintegrated
     );
  
      // Predict initial estimates for current state // **
      NavState prev_robot_state = NavState(prev_robot_pose, prev_robot_velocity);
      NavState propagated_robot_state = imu_preintegrated->predict(prev_robot_state, prev_robot_bias);
      newNodes.insert(Symbol('x', pose_id), propagated_robot_state.pose()); 
      newNodes.insert(Symbol('v', pose_id), propagated_robot_state.v()); 
      newNodes.insert(Symbol('b', pose_id), prev_robot_bias); 

      // UPDATE ISAM WITH NEW FACTORS AND NODES FROM THIS POSE 
      isam->update(graph, newNodes); 
      
//      // Print graph to graphviz dot file (render to PDF using "fdp filname.dot -Tpdf > filename.pdf")
//      if (pose_id == 2) {
//        ofstream os("/home/vkopli/Documents/GRASP/Graphs/VisualISAMActualGraphIMU_2pose_2019-09-18.dot");
//        graph.saveGraph(os, newNodes);
//        isam->saveGraph("/home/vkopli/Documents/GRASP/Graphs/VisualISAMGraphIMU_2pose_2019-09-18.dot");
//      }

//      // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
//      // If accuracy is desired at the expense of time, update(*) can be called additional times
//      // to perform multiple optimizer iterations every step.
//      isam->update();
//      ROS_INFO("after second update step");

      // Update the node values that have been seen up to this point
      optimizedNodes = isam->calculateEstimate();
//      optimizedNodes.print("Current estimate: ");

      // Clear the objects holding new factors and node values for the next iteration
      graph.resize(0);
      newNodes.clear();

       // Reset the IMU preintegration object // **
      imu_preintegrated->resetIntegrationAndSetBias(prev_robot_bias); 
          
      // Get optimized nodes for next iteration
      prev_robot_pose = optimizedNodes.at<Pose3>(Symbol('x', pose_id));
      prev_robot_velocity = optimizedNodes.at<Vector3>(Symbol('v', pose_id));
      prev_robot_bias = optimizedNodes.at<imuBias::ConstantBias>(Symbol('b', pose_id));
      
      ROS_INFO("frame %d", pose_id);
      cout << "Pose Estimate: \n" << prev_robot_pose << endl;
      cout << "Velocity Estimate: \n" << prev_robot_velocity << endl;
      cout << "Bias Estimate: \n" << prev_robot_bias << endl;
    }

    prev_imu_timestamp = imu_msg->header.stamp;
    pose_id++;
    
    publishTf(prev_robot_pose, prev_imu_timestamp);
  }
  
  void publishTf(Pose3 &robot_pose, ros::Time &timestamp) {
    
    tf::Quaternion q_tf;
    tf::Vector3 t_tf;
    tf::quaternionEigenToTF(robot_pose.rotation().toQuaternion(), q_tf);
    tf::vectorEigenToTF(robot_pose.translation(), t_tf);
    tf::Transform world_to_imu_tf = tf::Transform(q_tf, t_tf);
    tf_pub.sendTransform(tf::StampedTransform(
          world_to_imu_tf, timestamp, lv.world_frame_id, lv.robot_frame_id));
  }
  
  void initializeIMUParameters(const ImuConstPtr& imu_msg) { // **
    
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
    imu_preintegrated = new PreintegratedCombinedMeasurements(p, imuBias::ConstantBias()); // CHANGE BACK TO COMBINED (Combined<->Imu)
  }

};

// MAIN (same as isam2_imu.cpp)
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
