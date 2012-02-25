/**
 * @file   wintracker_remapper_node.cpp
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
 *
 * @brief Launces a node remapping sensor readings from the WinTracker to the gazebo/set_model_state
 * topic
 *
 *
 */

#include <ros/ros.h>

#include "wintracker_remapper.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "wintracker_remapper");

  WinTrackerRemapper remapper;
  ros::spin();
  return 0;
}
