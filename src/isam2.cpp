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

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
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

// GLOBAL VARIABLES
/* ************************************************************************* */

// Create a Factor Graph and Values to hold the new data, accessible from callback function
NonlinearFactorGraph graph;
Values initialEstimate;


// FUNCTIONS
/* ************************************************************************* */
void callback(const CameraMeasurementConstPtr& features, const ImuConstPtr& imu) {
  // use features to estimate feature locations in camera frame
  // 
}

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/* ************************************************************************* */
int main(int argc, char **argv) {


  ros::init(argc, argv, "isam2_node"); // specify name of node and ROS arguments

  ros::NodeHandle nh;

  // Subscriber stays instantiated as long as subscribed, specify callback here
  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  message_filters::Subscriber<CameraMeasurement> feature_sub(nh, "features", 1);
  message_filters::Subscriber<Imu> imu_sub(nh, "imu", 1);
  TimeSynchronizer<CameraMeasurement, Imu> sync(feature_sub, imu_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // loop, pumping all callbacks (specified in subscriber object)
  ros::spin(); 

  return 0;
}

