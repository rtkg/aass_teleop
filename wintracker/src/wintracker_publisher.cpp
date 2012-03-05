#include "wintracker_publisher.h"
#include "WTracker.h"
#include <ros/ros.h>
#include <stdlib.h>
#include <cmath>
//-------------------------------------------------------------------------------------
WintrackerPublisher::WintrackerPublisher() : nh_("~"), frame_id_("/fixed")
{                                            
  std::string prefix;
  std::string frame_id;
  std::string searched_param;

  nh_.searchParam("hemisphere_specification",searched_param);
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
}
//-------------------------------------------------------------------------------------
WintrackerPublisher::~WintrackerPublisher()
{
 ROS_INFO("Shutting down the WinTracker node");
 shutdown_wtracker();
}
//-------------------------------------------------------------------------------------
bool WintrackerPublisher::getPose(wintracker::GetPose::Request  &req, wintracker::GetPose::Response &res)
{

  //Reads only the sensor on the first serial port - could be changed via publishing a pose array
  res.pose_stamped=getFilteredTick();

  res.success=true;
  return res.success;
}
//-------------------------------------------------------------------------------------
void WintrackerPublisher::startWTracker() 
{
  data_mutex_.lock();

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
      ROS_WARN("No Hemisphere specified - previous settings are used.");

  // enable_cont_mode();//Enables the WinTracker to continously send data

  //Fill up the sign buffer and generate an initial reference posture
  while((getCurrPos().norm() < 0.001) || (getCurrOri().norm() < 0.001) ) //make sure that the tracker is active
    tick_wtracker();

  sign_buffer_.resize(7,JITTER_WINDOW);

  for (int j=JITTER_WINDOW-1; j >= 0; --j)
    {
      tick_wtracker();
      sign_buffer_(0,j)=signof((float)wtrackerSensors[0].x);  
      sign_buffer_(1,j)=signof((float)wtrackerSensors[0].y); 
      sign_buffer_(2,j)=signof((float)wtrackerSensors[0].z);  
      sign_buffer_(3,j)=signof((float)wtrackerSensors[0].qx); 
      sign_buffer_(4,j)=signof((float)wtrackerSensors[0].qy); 
      sign_buffer_(5,j)=signof((float)wtrackerSensors[0].qz);  
      sign_buffer_(6,j)=signof((float)wtrackerSensors[0].qw);

      if(j==0)
	{
	  posture_ref_.head(3)=getCurrPos();
	  posture_ref_.tail(4)=getCurrOri();
	}
    }

  data_mutex_.unlock();
}
//-------------------------------------------------------------------------------------
bool WintrackerPublisher::spin() {
  while (ros::ok()) 
   {

    pub_.publish(getFilteredTick());

    ros::spinOnce();
  }
  return true ;
}
//-------------------------------------------------------------------------------------
Eigen::Vector4f WintrackerPublisher::getCurrOri()
{
 return Eigen::Vector4f((float)wtrackerSensors[0].qx,(float)wtrackerSensors[0].qy,(float)wtrackerSensors[0].qz,(float)wtrackerSensors[0].qw);
}
//-------------------------------------------------------------------------------------
Eigen::Vector3f WintrackerPublisher::getCurrPos()
{
 return Eigen::Vector3f((float)wtrackerSensors[0].x,(float)wtrackerSensors[0].y,(float)wtrackerSensors[0].z);
}
//-------------------------------------------------------------------------------------
geometry_msgs::PoseStamped WintrackerPublisher::getFilteredTick()
{
  Eigen::Vector4f curr_ori=getCurrOri();
  Eigen::Vector3f curr_pos=getCurrPos();

  data_mutex_.lock();

  tick_wtracker();

  filterSignJitter(curr_pos,curr_ori);

  data_mutex_.unlock(); 

  geometry_msgs::PoseStamped ps;

  ps.pose.position.x = curr_pos(0);
  ps.pose.position.y = curr_pos(1);
  ps.pose.position.z = curr_pos(2);
  ps.pose.orientation.x = curr_ori(0);
  ps.pose.orientation.y = curr_ori(1);
  ps.pose.orientation.z = curr_ori(2);
  ps.pose.orientation.w = curr_ori(3);
  ps.header.frame_id = frame_id_;

  return ps;
}
//-------------------------------------------------------------------------------------
bool WintrackerPublisher::signsEqual(Eigen::VectorXi const& vec)
{

  for(unsigned int i=0;i<vec.size()-1;i++)
    if(vec(i)*vec(i+1) < 0)
      return false;

  return true;
}
//-------------------------------------------------------------------------------------
void WintrackerPublisher::filterSignJitter(Eigen::Vector3f& pos, Eigen::Vector4f& ori)
{
  sign_buffer_.topLeftCorner(7,JITTER_WINDOW-1)=sign_buffer_.topRightCorner(7, JITTER_WINDOW-1).eval(); //pop the first column
  sign_buffer_.block(0,JITTER_WINDOW-1,3,1)=signof(pos); //push back
  sign_buffer_.block(3,JITTER_WINDOW-1,4,1)=signof(ori); //push back

  bool set_ref=true;

  for (unsigned int i=0; i < 3; i++)  
    if(!signsEqual(sign_buffer_.row(i)))
      set_ref=false;        

  if (set_ref)  //Update the reference posture if all signs in the queue were the same
    posture_ref_.head(3)=pos;
  else
    pos=posture_ref_.head(3);  //Freeze the posture to the reference if not all signs in the queue were the same
  set_ref=true;

  for (unsigned int i=0; i < 4; i++)  
    if(!signsEqual(sign_buffer_.row(i+3)))
      set_ref=false;        

  if (set_ref)
    posture_ref_.tail(4)=ori;
  else
    ori=posture_ref_.tail(4); 
}
//---------------------------------------------------------------------------
int WintrackerPublisher::signof(float val) 
{
  if(val < 0.0f)
    return -1;
  else
    return 1; 
}
//---------------------------------------------------------------------------
Eigen::VectorXi WintrackerPublisher::signof(Eigen::VectorXf vec) 
{
  unsigned int size=vec.size();

  Eigen::VectorXi signs(size);
  signs.setOnes(size);

  for (unsigned int i=0; i<size; i++)
    if(vec(i) < 0.0f)
      signs(i)=(-1);

      return signs;
}
//---------------------------------------------------------------------------
