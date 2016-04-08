/*
 * Node for Vision tracker.
 *
 *  Created on: April 5, 2016
 *      Author: 062.127@gmail.com
 */


#include <ros/ros.h>
#include "../main/main_kcf.hpp"
#include <stdio.h>
#include <thread>


int main(int argc, char **argv){

  bool stop_flag = false;

  ros::init(argc, argv, "vision_tracker");

  KcfTrackerRun mainObj;

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("uwb_coordinates", 1, &TrackerRun::getUWBMessages, dynamic_cast<TrackerRun*>(&mainObj));
  //UWBTracker uwb;

  /*
  if (!mainObj.start(argc, argv)) {
    return -1;
  }
  */

  std::thread vision_thread(&KcfTrackerRun::start, &mainObj, argc, argv, &stop_flag);

  ros::Rate r(10);
  while(!stop_flag) { // If thread is still running
    ros::spinOnce();
  }
  vision_thread.join();
  //ros::spin();


  return(0);
}
