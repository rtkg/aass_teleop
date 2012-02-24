#include "wintracker_publisher.h"
#include "WTracker.h"
#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

WintrackerPublisher::WintrackerPublisher() : nh_("~") {
  std::string prefix;
  std::string searched_param;
  // searches for parameter with name containing 'cyberglove_prefix' 
  if (nh_.searchParam("wintracker_prefix", searched_param) ) 
    nh_.getParam(searched_param, prefix);

  std::string full_topic = prefix + "/poseStamped";
  pubTest_ = nh_.advertise<geometry_msgs::PoseStamped>(full_topic, 2);

  //
  full_topic = prefix + "/pose";
  pub_ = nh_.advertise<geometry_msgs::Pose>(full_topic, 2);

  ros::Rate loop_rate(10);
}


void WintrackerPublisher::startWTracker() {
  if(initialize_wtracker() != 0) {
    ROS_ERROR("Failed to initialize wtracker, exiting");
    exit(0);
  }
}

void WintrackerPublisher::shutdownWTracker() {
  shutdown_wtracker();
}

bool WintrackerPublisher::spin() {
  while (nh_.ok()) {            // While the node has not been shutdown
    usleep(1) ;
    geometry_msgs::PoseStamped ps;
    geometry_msgs::Pose p;
    
    tick_wtracker();
    std::string fr_id = "/fixed"; ///< This is hardcoded fix to allow
				  ///visualisation in rvis when reference
				  ///frame /fixed is set

    printf("%+.3f %+.3f %+.3f\t\t",
	   wtrackerSensors[0].x,wtrackerSensors[0].y,wtrackerSensors[0].z);
    printf("%+.3f %+.3f %+.3f %+.3f\n",
	   wtrackerSensors[0].qx,wtrackerSensors[0].qy,
	   wtrackerSensors[0].qz,wtrackerSensors[0].qw);

    
    p.position.x = (float)wtrackerSensors[0].x;
    p.position.y = (float)wtrackerSensors[0].y;
    p.position.z = (float)wtrackerSensors[0].z;
    p.orientation.x = (float)wtrackerSensors[0].qx;
    p.orientation.y = (float)wtrackerSensors[0].qy;
    p.orientation.z = (float)wtrackerSensors[0].qz;
    p.orientation.w = (float)wtrackerSensors[0].qw;

    ps.header.frame_id = fr_id;
    ps.pose = p;

    pubTest_.publish(ps);
    pub_.publish(p);
    ros::spinOnce();
  }
  return true ;
}
