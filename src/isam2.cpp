// SUBSCRIBER NODE ("listener")

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info) {
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
  message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
  TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // loop, pumping all callbacks (specified in subscriber object)
  ros::spin(); 

  return 0;
}

