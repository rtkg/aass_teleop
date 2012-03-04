/**
 * @file   wintracker_remapper.cpp
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
 *
*
*/

#include "wintracker_remapper.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include "../srv_gen/cpp/include/wintracker/GetPose.h"
#include <std_srvs/Empty.h>

//---------------------------------------------------------------------------
WinTrackerRemapper::WinTrackerRemapper() : nh_private_("~"), gazebo_model_("gplane")
{
  std::string searched_param;
  if(!nh_private_.searchParam("gazebo_model", searched_param))
    ROS_WARN("No Gazebo model specified - using %s as remapping reference", gazebo_model_.c_str());
  else
    nh_private_.getParam(searched_param, gazebo_model_);

  // searches for parameter with name containing 'wintracker_prefix' 
  nh_private_.searchParam("wintracker_prefix", searched_param); 
  nh_private_.getParam(searched_param, wintracker_prefix_);

  nh_private_.searchParam("gazebo_prefix", searched_param); 
  nh_private_.getParam(searched_param, gazebo_prefix_);

   start_remap_srv_ = nh_.advertiseService(wintracker_prefix_ + "/start_remap",&WinTrackerRemapper::startRemap,this);
   stop_remap_srv_ = nh_.advertiseService(wintracker_prefix_ + "/stop_remap",&WinTrackerRemapper::stopRemap,this);
   gazebo_modstat_clt_ =  nh_.serviceClient<gazebo_msgs::GetModelState>(gazebo_prefix_ + "/get_model_state");

  gazebo_set_modstat_clt_ =  nh_.serviceClient<gazebo_msgs::SetModelState>(gazebo_prefix_ + "/set_model_state");
     gazebo_pause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix_ + "/pause_physics");


   wt_get_pose_clt_ =  nh_.serviceClient<wintracker::GetPose>(wintracker_prefix_ + "/get_pose");
   gazebo_unpause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix_ + "/unpause_physics");

  //Hardcoded pose offset between the sensor and the model - would be good to read this from the parameter server
  btMatrix3x3 M_R_S(0,-1,0,0,0,-1,1,0,0);
  M_T_S_.setBasis(M_R_S);
  M_T_S_.setOrigin(tf::Vector3(0,0,0)); 

}
//---------------------------------------------------------------------------
bool WinTrackerRemapper::startRemap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  //get the initial pose of the model from Gazebo
  gazebo_msgs::GetModelState gz_pose; 
  gz_pose.request.model_name=gazebo_model_;
  gz_pose.request.relative_entity_name="world";
  gazebo_modstat_clt_.call(gz_pose);
  if(!gz_pose.response.success)
    {
      ROS_ERROR("Could not get the state of model %s from gazebo. Cannot start remapping.",gazebo_model_.c_str());
      return false;
    }
 std::cout<<"got gz pose herer"<<std::endl;
  //get the initial pose from the WinTracker
  wintracker::GetPose wt_pose;
  wt_get_pose_clt_.call(wt_pose);
  if(!wt_pose.response.success)
    {
      ROS_ERROR("Could not get the pose from the WinTracker. Cannot start remapping.");
      return false;
    }
  std::cout<<"got wt pose herer"<<std::endl;
  tf::Transform G_T_M;//initital pose of the model in the Gazebo frame
  G_T_M.setOrigin(tf::Vector3(gz_pose.response.pose.position.x,gz_pose.response.pose.position.y,gz_pose.response.pose.position.z));
  G_T_M.setRotation(tf::Quaternion(gz_pose.response.pose.orientation.x,gz_pose.response.pose.orientation.y,gz_pose.response.pose.orientation.z,gz_pose.response.pose.orientation.w));

  tf::Transform W_T_S; //initital pose of the sensor in the WinTracker frame
  W_T_S.setOrigin(tf::Vector3(wt_pose.response.pose_stamped.pose.position.x,wt_pose.response.pose_stamped.pose.position.y,wt_pose.response.pose_stamped.pose.position.z));
  W_T_S.setRotation(tf::Quaternion(wt_pose.response.pose_stamped.pose.orientation.x,wt_pose.response.pose_stamped.pose.orientation.y,wt_pose.response.pose_stamped.pose.orientation.z,wt_pose.response.pose_stamped.pose.orientation.w)); 

  G_T_W_=G_T_M*M_T_S_*W_T_S.inverse(); //Transformation from the WinTracker to the Gazebo coordinate frame

 //Advertise & Subscribe
  model_state_pub_ = nh_.advertise<gazebo_msgs::ModelState> (gazebo_prefix_ + "/set_model_state", 2); 
  wintracker_poses_sub_ = nh_.subscribe(wintracker_prefix_ + "/pose", 2, &WinTrackerRemapper::remapPose, this);

  //Unpause the Gazebo 
  std_srvs::Empty empty;
  gazebo_unpause_clt_.call(empty);
  std::cout<<"IM herer"<<std::endl;
  data_mutex_.unlock();
  return true;
}
//---------------------------------------------------------------------------
bool WinTrackerRemapper::stopRemap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  data_mutex_.lock();

  std_srvs::Empty empty;
  gazebo_pause_clt_.call(empty);
  wintracker_poses_sub_.shutdown();
  model_state_pub_.shutdown();  

  data_mutex_.unlock();
  return true;
}
//---------------------------------------------------------------------------
void WinTrackerRemapper::remapPose(const geometry_msgs::PoseStamped & ps)
{
  gazebo_msgs::ModelState ms;
  tf::Transform G_T_M; //pose of the model in the Gazebo coordinate frame
  tf::Transform W_T_S; //pose of the sensor in the WinTracker frame
 
  W_T_S.setOrigin(tf::Vector3(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z));
  W_T_S.setRotation(tf::Quaternion(ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w));

  //compute the pose of the model expressed in the Gazebo frame
  G_T_M=G_T_W_*W_T_S*M_T_S_.inverse(); 

  //get the remapped pose
  tf::Vector3 rmp_v= G_T_M.getOrigin();
  tf::Quaternion rmp_q=G_T_M.getRotation();

  ms.model_name=gazebo_model_;
  ms.pose.position.x=rmp_v.x();
  ms.pose.position.y=rmp_v.y();
  ms.pose.position.z=rmp_v.z();
  ms.pose.orientation.x=rmp_q.x();
  ms.pose.orientation.y=rmp_q.y();
  ms.pose.orientation.z=rmp_q.z();
  ms.pose.orientation.w=rmp_q.w();
  ms.reference_frame="world";

  //model_state_pub_.publish(ms);
 
  gazebo_msgs::SetModelState sms;

  sms.request.model_state=ms;
  std_srvs::Empty empty;

  gazebo_pause_clt_.call(empty);

  gazebo_set_modstat_clt_.call(sms);
  
  gazebo_unpause_clt_.call(empty);


}
//---------------------------------------------------------------------------



