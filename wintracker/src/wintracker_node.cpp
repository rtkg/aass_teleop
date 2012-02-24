#include "wintracker_publisher.h"
#include <ros/ros.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "wintracker_node");

  //Constructing wintracker_node
  WintrackerPublisher wtracker_publisher;

  //Connect to windtracker_node
  wtracker_publisher.startWTracker();
  ROS_INFO("WinTracker node started");

  wtracker_publisher.spin() ;

  ROS_INFO("Shutting down the WinTracker node");
  wtracker_publisher.shutdownWTracker() ;

  return 0 ;
}
