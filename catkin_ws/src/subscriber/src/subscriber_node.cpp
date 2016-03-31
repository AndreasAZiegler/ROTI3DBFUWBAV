#include "ros/ros.h"
#include "uwb_dummy/Coordinates.h"

void subscriberCallback(const uwb_dummy::Coordinates::ConstPtr& msg) {
  ROS_INFO("I heard: [%d]", msg->coordinates[0]);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("uwb_coordinates", 1000, subscriberCallback);

  ros::spin();

  return(0);
}
