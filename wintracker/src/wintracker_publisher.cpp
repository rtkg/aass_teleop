#include "wintracker_publisher.h"
#include "WTracker.h"
#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/PoseStamped.h"

WintrackerPublisher::WintrackerPublisher() : nh_("~"), frame_id_("/fixed"), hemisphere_("Up"){
  std::string prefix;
  std::string frame_id;
  std::string searched_param;

  if(nh_.searchParam("hemisphere_specification",searched_param))
    nh_.getParam(searched_param, hemisphere_);

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
  pose_srv_=nh_.advertiseService(prefix + "/get_pose",&WintrackerPublisher::getPose,this);
  ros::Rate loop_rate(10);
}
bool WintrackerPublisher::getPose(wintracker::GetPose::Request  &req, wintracker::GetPose::Response &res)
{
  res.success=false;
 
  data_mutex_.lock();
  tick_wtracker();
  data_mutex_.unlock();   

  //Reads only the sensor on the first serial port - could be changed via publishing a pose array
  res.pose_stamped.pose.position.x = (float)wtrackerSensors[0].x;
  res.pose_stamped.pose.position.y = (float)wtrackerSensors[0].y;
  res.pose_stamped.pose.position.z = (float)wtrackerSensors[0].z;
  res.pose_stamped.pose.orientation.x = (float)wtrackerSensors[0].qx;
  res.pose_stamped.pose.orientation.y = (float)wtrackerSensors[0].qy;
  res.pose_stamped.pose.orientation.z = (float)wtrackerSensors[0].qz;
  res.pose_stamped.pose.orientation.w = (float)wtrackerSensors[0].qw;

  res.pose_stamped.header.frame_id = frame_id_;

  res.success=true;
  return res.success;
}
void WintrackerPublisher::startWTracker() 
{

  if(initialize_wtracker() != 0) 
  {
    ROS_ERROR("Failed to initialize wtracker, exiting");
    exit(0);
  }

  if(hemisphere_=="Up")
    setUpHemisphere();
  else if(hemisphere_=="Front")
    setFrontHemisphere();
  else
    {
      ROS_WARN("Hemisphere specification '%s' is invalid. It has to be either 'Up' or 'Front'. Setting hemisphere to 'Up'.",hemisphere_.c_str());
      setUpHemisphere();
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
    
    data_mutex_.lock();
    tick_wtracker();
    data_mutex_.unlock();   

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
