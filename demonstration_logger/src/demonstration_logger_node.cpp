/**
 *
 * @brief 
 *
 *
 */

#include <ros/ros.h>
#include "demonstration_logger/demonstration_logger.h"

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "demonstration_logger");

  DemonstrationLogger demonstration_logger;
  ros::spin();

  return 0;
}

