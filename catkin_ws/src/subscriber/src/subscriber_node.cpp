#include "ros/ros.h"
#include "uwb/UWBTracker.h"

void subscriberCallback(const uwb::UWBTracker::ConstPtr& msg) {
  ROS_INFO("I heard: [%f]", msg->state[1]);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("UWB_Tracker", 1000, subscriberCallback);

  ros::spin();

  return(0);
}
