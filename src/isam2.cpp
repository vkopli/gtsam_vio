// SUBSCRIBER NODE ("isam2")

// ROS/PACKAGE INCLUDES
/* ************************************************************************* */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <legged_vio/CameraMeasurement.h>
#include <sensor_msgs/Imu.h>

// ISAM2 INCLUDES
/* ************************************************************************* */

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

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
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// ADDITIONAL INCLUDES
/* ************************************************************************* */

#include <vector>

using namespace message_filters;
using namespace legged_vio;
using namespace sensor_msgs;
using namespace gtsam;


// CALLBACK WRAPPER CLASS
/* ************************************************************************* */

class Callbacks { 

private:
  
  // Hold node handle initialized in main
  ros::NodeHandle nh;
  
  // Create a Factor Graph and Values to hold the new data, accessible from callback function
  NonlinearFactorGraph graph;
  Values initialEstimate;
  

public:
  Callbacks(ros::NodeHandle& nh) : nh(nh) {

    // Initialize Camera Calibration Constants from YAML file
    // nh.getParam("cam0/resolution", cam0_resolution_temp);
  }

  void callback(const CameraMeasurementConstPtr& features, const ImuConstPtr& imu) {
  
    // Use features u, v image coordinates to estimate feature X, Y, Z locations in camera frame
    // maybe with openCV, maybe something already in msckf_vio  

    // Create Point3 objects (feature in camera -> feature in world) 

    // Create Pose3 object (estimated camera pose)  

    // Add to Factor Graph

    // Optimize using ISAM

    // Print Results to ROS_INFO
  }

};

/* ************************************************************************* */
int main(int argc, char **argv) {

  ros::init(argc, argv, "isam2_node"); // specify name of node and ROS arguments
  ros::NodeHandle nh;

  // Instantiate class containing callbacks and necessary variables
  Callbacks callbacks_obj(nh);

  // Subscribe to "features" and "imu" topics simultaneously
  message_filters::Subscriber<CameraMeasurement> feature_sub(nh, "features", 1);
  message_filters::Subscriber<Imu> imu_sub(nh, "imu", 1);
  TimeSynchronizer<CameraMeasurement, Imu> sync(feature_sub, imu_sub, 10);
  sync.registerCallback(boost::bind(&Callbacks::callback, &callbacks_obj, _1, _2));

  // loop, pumping all callbacks (specified in subscriber object)
  ros::spin(); 

  return 0;
}

