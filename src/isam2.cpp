// SUBSCRIBER NODE ("listener")

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <legged_vio/CameraMeasurement.h>
#include <sensor_msgs/Imu.h>

using namespace message_filters;
using namespace legged_vio;
using namespace sensor_msgs;

void callback(const CameraMeasurementConstPtr& features, const ImuConstPtr& imu) {
  //
}

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

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

