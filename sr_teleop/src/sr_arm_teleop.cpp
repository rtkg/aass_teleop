/**
 * @file   sr_arm_teleop.cpp
 * @author Robert Krug
 * @date   Tue, Mar 4, 2012
 *
*
*/

#include "sr_arm_teleop.h"
#include <wintracker/GetPose.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <boost/algorithm/string.hpp>

//---------------------------------------------------------------------------
SrArmTeleop::SrArmTeleop() : nh_private_("~")
{
  std::string searched_param;
  std::string sr_teleop_prefix;
  std::string cart_controller_prefix;
  std::string sensor_prefix;

  nh_private_.searchParam("cartesian_pose_controller", searched_param);
  nh_private_.getParam(searched_param, cart_pose_controller_);

  nh_private_.searchParam("base_frame_id", searched_param); 
  nh_private_.getParam(searched_param, base_frame_id_);

  nh_private_.searchParam("track_frame_id", searched_param); 
  nh_private_.getParam(searched_param, track_frame_id_);
  

  //initialize the pose offset between the sensor and the tracked object
  T_T_S_.setIdentity();

  //see if a pose offset T_T_S is specified on the parameter server
  XmlRpc::XmlRpcValue T_T_S;
  if (nh_private_.searchParam("T_T_S", searched_param))
    {
      nh_.getParam(searched_param, T_T_S);
      ROS_ASSERT(T_T_S.getType() == XmlRpc::XmlRpcValue::TypeArray); 
      ROS_ASSERT(T_T_S.size()==12);//a pose is specified by a rotation matrix + offset vector
      for (int32_t i = 0; i < T_T_S.size(); ++i) 
	ROS_ASSERT(T_T_S[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

      T_T_S_.setBasis(btMatrix3x3(T_T_S[0],T_T_S[1],T_T_S[2],T_T_S[3],T_T_S[4],T_T_S[5],T_T_S[6],T_T_S[7],T_T_S[8]));
      T_T_S_.setOrigin(tf::Vector3(T_T_S[9],T_T_S[10],T_T_S[11]));
    }
  else
    ROS_INFO("No local pose offset T^T_S between the sensor and the tracked link is applied.");


  //get the default controllers from the parameter server - those will be switched with the cartesian pose controller
  XmlRpc::XmlRpcValue default_controllers;
  if (nh_private_.searchParam("default_controllers", searched_param))
    {
      nh_.getParam(searched_param, default_controllers);
      ROS_ASSERT(default_controllers.getType() == XmlRpc::XmlRpcValue::TypeArray); 
      for (int32_t i = 0; i < default_controllers.size(); ++i) 
	{
      	ROS_ASSERT(default_controllers[i].getType() == XmlRpc::XmlRpcValue::TypeString);
        default_controllers_.push_back(default_controllers[i]);
	}
    }
  else
    ROS_WARN("No default controllers were specified.");

  //Initialize B_T_W_, the transform from the emitter to the base coordinate frame
  B_T_E_.setIdentity();

   start_teleop_srv_ = nh_.advertiseService("start_teleop",&SrArmTeleop::startTeleop,this);
   stop_teleop_srv_ = nh_.advertiseService("stop_teleop",&SrArmTeleop::stopTeleop,this);
   pose_setpt_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command",1);


   get_sensor_pose_clt_= nh_.serviceClient<wintracker::GetPose>("get_pose");
   switch_ctrl_clt_ = nh_.serviceClient<pr2_mechanism_msgs::SwitchController>("switch_controller");

#ifdef DEBUG
   sensor_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("sensor_pose",1);
#endif

}
//-------------------------------------------------
void SrArmTeleop::sensorCallback(const geometry_msgs::PoseStamped & ps)
{
    
  geometry_msgs::PoseStamped track_ps;
  tf::Transform B_T_T; //current pose of the tracked object expressed in the base coordinate frame
  tf::Transform E_T_S; //current pose of the sensor expressed in the emitter frame
 
  E_T_S.setOrigin(tf::Vector3(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z));
  E_T_S.setRotation(tf::Quaternion(ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w));

  //compute the current pose of the tracked object expressed in the base frame
  B_T_T=B_T_E_*E_T_S*T_T_S_.inverse(); 

  //construct  the remapped pose
  tf::Vector3 rmp_v= B_T_T.getOrigin();
  tf::Quaternion rmp_q=B_T_T.getRotation();

 
  track_ps.header.frame_id="/"+getRelativeName(base_frame_id_);
  track_ps.pose.position.x=rmp_v.x();
  track_ps.pose.position.y=rmp_v.y();
  track_ps.pose.position.z=rmp_v.z();
  track_ps.pose.orientation.x=rmp_q.x();
  track_ps.pose.orientation.y=rmp_q.y();
  track_ps.pose.orientation.z=rmp_q.z();
  track_ps.pose.orientation.w=rmp_q.w();
  track_ps.header.seq=ps.header.seq;
  track_ps.header.stamp=ps.header.stamp;

  //publish
   pose_setpt_pub_.publish(track_ps);

#ifdef DEBUG //publish the pose of the sensor in the base frame for debugging
   tf::Transform B_T_S=B_T_E_*E_T_S; 
   rmp_v= B_T_S.getOrigin();
   rmp_q=B_T_S.getRotation();
   geometry_msgs::PoseStamped sensor_ps;

   sensor_ps.header.frame_id=base_frame_id_;
   sensor_ps.pose.position.x=rmp_v.x();
   sensor_ps.pose.position.y=rmp_v.y();
   sensor_ps.pose.position.z=rmp_v.z();
   sensor_ps.pose.orientation.x=rmp_q.x();
   sensor_ps.pose.orientation.y=rmp_q.y();
   sensor_ps.pose.orientation.z=rmp_q.z();
   sensor_ps.pose.orientation.w=rmp_q.w();
   sensor_ps.header.seq=ps.header.seq;
   sensor_ps.header.stamp=ps.header.stamp;

   sensor_pose_pub_.publish(sensor_ps);
#endif
}
//-------------------------------------------------
bool SrArmTeleop::startTeleop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
 
  ROS_INFO("Synchronizing sensor pose and tracked link pose ...");

  lock_.lock();

  //get the current pose from the sensor expressed in the emitter frame
  wintracker::GetPose s_ps;
  get_sensor_pose_clt_.call(s_ps);
  if(!s_ps.response.success)
    {
      ROS_ERROR("Could not get the sensor pose - cannot synchronize.");
      return false;
    }
  tf::Transform E_T_S; 
  E_T_S.setOrigin(tf::Vector3(s_ps.response.pose_stamped.pose.position.x,s_ps.response.pose_stamped.pose.position.y,s_ps.response.pose_stamped.pose.position.z));
  E_T_S.setRotation(tf::Quaternion(s_ps.response.pose_stamped.pose.orientation.x,s_ps.response.pose_stamped.pose.orientation.y,s_ps.response.pose_stamped.pose.orientation.z,s_ps.response.pose_stamped.pose.orientation.w)); 

  //get the current pose of the tracked link expressed in the base frame
  tf::StampedTransform B_T_T;
  ros::Duration(0.5).sleep(); //sleep for half a second to make sure the TransformListener's buffer is not empty
  try
  {
    tf_list_.waitForTransform(base_frame_id_, track_frame_id_,s_ps.response.pose_stamped.header.stamp, ros::Duration(1.0));
    tf_list_.lookupTransform(base_frame_id_, track_frame_id_,s_ps.response.pose_stamped.header.stamp ,B_T_T);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    lock_.unlock(); 
    return false;
  }

  B_T_E_=B_T_T*T_T_S_*E_T_S.inverse(); //setting the static transformation from the emitter to the base_link coordinate frame

  lock_.unlock();

  ROS_INFO("Switching control mode - Entering teleoperation mode ...");

  pr2_mechanism_msgs::SwitchController switch_ctrl;
  switch_ctrl.request.start_controllers=std::vector<std::string>(1,cart_pose_controller_);
  switch_ctrl.request.stop_controllers=default_controllers_;
  switch_ctrl.request.strictness=2;//strict rule

  switch_ctrl_clt_.call(switch_ctrl);

  if(!switch_ctrl.response.ok)
     {
      ROS_ERROR("Could not switch controllers - cannot start teleoperating mode");
      return false;
    }

  //Subscribe to the sensor data publisher - the remapping from sensor poses to tracked link poses is done in the sensorCallback
    sensor_poses_sub_ = nh_.subscribe("pose", 2, &SrArmTeleop::sensorCallback, this); 

  return true;
}
//-------------------------------------------------
bool SrArmTeleop::stopTeleop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
   ROS_INFO("Switching control mode - Leaving teleoperation mode ...");

  pr2_mechanism_msgs::SwitchController switch_ctrl;
  switch_ctrl.request.start_controllers= default_controllers_;
  switch_ctrl.request.stop_controllers=std::vector<std::string>(1,cart_pose_controller_);
  switch_ctrl.request.strictness=2;//strict rule

  switch_ctrl_clt_.call(switch_ctrl);

  if(!switch_ctrl.response.ok)
     {
      ROS_ERROR("Could not switch controllers - cannot leave teleoperating mode");
      return false;
    }

  //stop remapping the poses from the sensor to the controller
   sensor_poses_sub_.shutdown();
  
  return true;
}
//---------------------------------------------------------------------------
std::string SrArmTeleop::getRelativeName(std::string & name)
{
  if(name.size()==0)
    return "";

  std::vector<std::string> splitted_name;
  boost::split(splitted_name, name, boost::is_any_of("/"));

  return splitted_name[splitted_name.size()-1];
}
//---------------------------------------------------------------------------
