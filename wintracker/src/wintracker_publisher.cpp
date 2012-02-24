#include "wintracker_publisher.h"
#include "WTracker.h"
#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/PoseStamped.h"

WintrackerPublisher::WintrackerPublisher() : nh_("~"), frame_id_("/fixed") {
  std::string prefix;
  std::string frame_id;
  std::string searched_param;

  // searches for parameter with name containing 'wintracker_prefix' 
  nh_.searchParam("wintracker_prefix", searched_param); 
  nh_.getParam(searched_param, prefix);

 if (nh_.searchParam("wintracker_frame_id", searched_param)) 
   {
    nh_.getParam(searched_param, frame_id);
    frame_id_=frame_id;
   }

  std::string full_topic = prefix + "/pose";
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>(full_topic, 2);

  ros::Rate loop_rate(10);
}


void WintrackerPublisher::startWTracker() 
{
  if(initialize_wtracker() != 0) 
  {
    ROS_ERROR("Failed to initialize wtracker, exiting");
    exit(0);
  }
}

void WintrackerPublisher::shutdownWTracker() 
{
  shutdown_wtracker();
}

bool WintrackerPublisher::spin() {
  while (nh_.ok()) {            // While the node has not been shutdown
    usleep(1) ;
    geometry_msgs::PoseStamped ps;
    
    tick_wtracker();

    //Reads only the sensor on the first serial port - could be changed via publishing a pose array
    ps.pose.position.x = (float)wtrackerSensors[0].x;
    ps.pose.position.y = (float)wtrackerSensors[0].y;
    ps.pose.position.z = (float)wtrackerSensors[0].z;
    ps.pose.orientation.x = (float)wtrackerSensors[0].qx;
    ps.pose.orientation.y = (float)wtrackerSensors[0].qy;
    ps.pose.orientation.z = (float)wtrackerSensors[0].qz;
    ps.pose.orientation.w = (float)wtrackerSensors[0].qw;

    ps.header.frame_id = frame_id_;

    pub_.publish(ps);
    ros::spinOnce();
  }
  return true ;
}
