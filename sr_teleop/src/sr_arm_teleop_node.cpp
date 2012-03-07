/**
 * @file   sr_arm_teleop_node.cpp
 * @author Robert Krug
 * @date   Tue Mar 6, 2012
 *
 * @brief Launches a node for teleoperating the Shadow hand/arm via a Cyberglove and a WinTracker 6D pose sensor
 *
 *
 */

#include <ros/ros.h>

#include "sr_arm_teleop.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_teleop");

  SrArmTeleop sr_arm_teleop;
  ros::spin();
  return 0;
}
