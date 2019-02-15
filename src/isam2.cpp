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

#include <vector>
#include <memory>

using namespace std;
using namespace message_filters;
using namespace legged_vio;
using namespace sensor_msgs;
using namespace gtsam;


// CALLBACK WRAPPER CLASS
/* ************************************************************************* */

class Callbacks { 

private:
  
	int frame = 0;

  // Hold node handle initialized in main
  shared_ptr<ros::NodeHandle> nh_ptr;
  
  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph graph;
  Values initial_estimate; // initial estimates of feature locations (factors)

  // --> Create iSAM2 object
  unique_ptr<ISAM2> isam;

	// Camera calibration intrinsics
	double f;
	double cx;
	double cy;

  // Camera calibration intrinsic matrix
  Cal3_S2Stereo::shared_ptr K;

  // Camera calibration extrinsic
  double Tx; // distance from cam0 to cam1

  // --> Camera observation noise model (has to do with IMU?)
  noiseModel::Isotropic::shared_ptr noise_model = 
			noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

public:
 
  Callbacks(shared_ptr<ros::NodeHandle> nh_ptr_copy) : nh_ptr(move(nh_ptr_copy)) {

    // YAML intrinsics (pinhole): [fu fv pu pv]
    vector<int> cam0_intrinsics(4);
    vector<int> cam1_intrinsics(4);
    nh_ptr->getParam("cam0/intrinsics", cam0_intrinsics); 
    nh_ptr->getParam("cam1/intrinsics", cam1_intrinsics);
		this->f = (cam0_intrinsics[0] + cam0_intrinsics[1]) / 2;
		// neglecting image center of right camera...
		this->cx = cam0_intrinsics[2];
		this->cy = cam0_intrinsics[3];
		
    // K: (fx, fy, s, u0, v0, b) (b: baseline where Z = f*d/b; Tx is negative)
		Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(cam0_intrinsics[0], 
			cam0_intrinsics[1], 0.0, cx, cy, -Tx)); 
		
    // YAML extrinsics (distance between 2 cameras)
		vector<double> T_cam1(16);
		nh_ptr->getParam("cam1/T_cn_cnm1", T_cam1);
		this->Tx = T_cam1[3];
    
    // iSAM2 settings
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam.reset(new ISAM2(parameters));

		// print to confirm reading the YAML file correctly
		ROS_INFO("cam0/intrinsics exists? %d", nh_ptr->hasParam("cam0/intrinsics")); 
		ROS_INFO("intrinsics: %d, %d, %d, %d", cam0_intrinsics[0], 
							cam0_intrinsics[1], cam0_intrinsics[2], cam0_intrinsics[3]);
    ROS_INFO("Tx: %f", Tx);

  }

  void callback(const CameraMeasurementConstPtr& camera_msg, const ImuConstPtr& imu_msg) {
  
    vector<FeatureMeasurement> feature_vector = camera_msg->features;  
    ROS_INFO("%lu total features", feature_vector.size());

    for (int i = 0; i < feature_vector.size(); i++) {

			 // initial estimate for pose
			initial_estimate.insert(Symbol('x', frame), Pose3());

			// initial estimate for landmarks
			processFeature(feature_vector[i]);

			if (frame == 0) {

      	// Add a prior on pose x0
				// 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
      	noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 				Vector3::Constant(0.3),Vector3::Constant(0.1)).finished()); 
      	graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), Pose3(), poseNoise);

      	// Should add prior on landmark l0?

			} else {

				// Update iSAM with the new factors
      	isam->update(graph, initial_estimate);

      	// Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
      	// If accuracy is desired at the expense of time, update(*) can be called additional times
      	// to perform multiple optimizer iterations every step.
      	isam->update();

				// estimate for current frame
      	Values current_estimate = isam->calculateEstimate();
        // Print Results to ROS_INFO

      	// Clear the factor graph and values for the next iteration
      	graph.resize(0);
      	initial_estimate.clear();
			}

			frame++;
		}

  }

	void processFeature(FeatureMeasurement feature) {

			// identify feature (may appear in previous/future frames)
			int l = feature.id; 

			double uL = feature.u0;
			double uR = feature.u1;
			double v = (feature.v0 + feature.v1) / 2;

			double d = uR - uL;
			double x = uL;
			double y = v;
      double W = -d / Tx;

			// estimated feature location in camera frame
			double X = (x - cx) / W;
			double Y = (y - cy) / W;
			double Z = f / W;
			
    	graph.emplace_shared<
      	  GenericStereoFactor<Pose3, Point3> >(StereoPoint2(uL, uR, v), 
						noise_model, Symbol('x', frame), Symbol('l', l), K);

			// add initial estimate of landmark if it hasn't appeared yet
	    if (!initial_estimate.exists(Symbol('l', l))) {
      		Pose3 camPose = initial_estimate.at<Pose3>(Symbol('x', frame));
      		Point3 worldPoint = camPose.transform_from(Point3(X, Y, Z)); 
      		initial_estimate.insert(Symbol('l', l), worldPoint);
    	}	
	}

};

// MAIN
/* ************************************************************************* */
int main(int argc, char **argv) {

  ros::init(argc, argv, "isam2"); // specify name of node and ROS arguments
  shared_ptr<ros::NodeHandle> nh_ptr = make_shared<ros::NodeHandle>();

  // Instantiate class containing callbacks and necessary variables
  Callbacks callbacks_obj(nh_ptr);

  // Subscribe to "features" and "imu" topics simultaneously
  message_filters::Subscriber<CameraMeasurement> feature_sub(*nh_ptr, "/minitaur/image_processor/features", 1);
  message_filters::Subscriber<Imu> imu_sub(*nh_ptr, "/imu0", 1);
  TimeSynchronizer<CameraMeasurement, Imu> sync(feature_sub, imu_sub, 10);
  sync.registerCallback(boost::bind(&Callbacks::callback, &callbacks_obj, _1, _2));

  // Loop, pumping all callbacks (specified in subscriber object)
  ros::spin(); 

  return 0;
}

// potential ROS messages
//ROS_INFO("IMU header: [%s]", imu_msg->header.frame_id);
//std_msgs::Header header = camera_msg->header;
//geometry_msgs::Quaternion ori = imu_msg->orientation;
//ROS_INFO("Camera Coor: [id = %d, u = %f, v = %f]", features[0].id, features[0].u0, features[0].v0);


