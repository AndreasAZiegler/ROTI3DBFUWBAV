//
// Created by andreasziegler on 4/5/16.
//

#include "../../3rdparty/cv_ext/tracker_run.hpp"
#include <ros/ros.h>
#include <iostream>
#include "uwb_dummy/Coordinates.h"

void TrackerRun::getUWBMessages(const uwb_dummy::Coordinates::ConstPtr &msg) {
  std::cout << "Message received." << std::endl;
  ROS_INFO("I heard: [%d]", msg->coordinates[0]);
}
