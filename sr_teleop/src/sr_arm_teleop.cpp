   /**
 * @file   sr_arm_teleop.cpp
 * @author Robert Krug
 * @date   Tue, Mar 4, 2012
 *
*
*/

#include "sr_arm_teleop.h"
#include <wintracker/GetPose.h>
#include <boost/algorithm/string.hpp>
#include "tf_conversions/tf_eigen.h"
//HACK
#include <sr_robot_msgs/sendupdate.h>
#include <sr_robot_msgs/joint.h>
//HACK END
//---------------------------------------------------------------------------
SrArmTeleop::SrArmTeleop() : nh_private_("~")
{
  std::string searched_param;
  std::string sr_teleop_prefix;
  std::string sensor_prefix;


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

  //Read the configuration from the parameter server
  nh_private_.searchParam("safety_zone_radius", searched_param);
  nh_private_.getParam(searched_param, sz_rad_);
  if(sz_rad_<=0)
    {
      sz_rad_=0.2;
      ROS_WARN("Invalid safety zone radius specified. Setting the radius to %f",sz_rad_);
    }

  //Initialize B_T_W_, the transform from the emitter to the base coordinate frame
  B_T_E_.setIdentity();

   start_teleop_srv_ = nh_.advertiseService("start_teleop",&SrArmTeleop::startTeleop,this);
   stop_teleop_srv_ = nh_.advertiseService("stop_teleop",&SrArmTeleop::stopTeleop,this);
   pose_setpt_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("track_pose",1);

   get_sensor_pose_clt_= nh_.serviceClient<wintracker::GetPose>("get_pose");
   //sensor_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("sensor_pose",1);


#ifdef DEBUG
  dbg_pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>("debug_pose",1);
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

 
  //check if the desired position is within the spherical safety zone and set it to a position on the boundary if not
  Eigen::Vector3d offset = Eigen::Vector3d(rmp_v.x(),rmp_v.y(),rmp_v.z())-tl_init_pos_;
  if(offset.norm() > sz_rad_)
    {
      ROS_WARN("Violating safety zone boundary - locking the position of the tracked link");
      tf::VectorEigenToTF(tl_init_pos_+sz_rad_*offset/offset.norm(), rmp_v);
    }
 
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

  //publish the pose of the tracked link in the base frame
  pose_setpt_pub_.publish(track_ps);

#ifdef DEBUG //Publish the sensor pose in the fully resolved base frame id for debugging
  tf::Transform B_T_S=B_T_E_*E_T_S; 
  rmp_v= B_T_S.getOrigin();
  rmp_q=B_T_S.getRotation();
  geometry_msgs::PoseStamped sensor_ps;

  sensor_ps.header.frame_id="/"+getRelativeName(base_frame_id_);
  sensor_ps.pose.position.x=rmp_v.x();
  sensor_ps.pose.position.y=rmp_v.y();
  sensor_ps.pose.position.z=rmp_v.z();
  sensor_ps.pose.orientation.x=rmp_q.x();
  sensor_ps.pose.orientation.y=rmp_q.y();
  sensor_ps.pose.orientation.z=rmp_q.z();
  sensor_ps.pose.orientation.w=rmp_q.w();
  sensor_ps.header.seq=ps.header.seq;
  sensor_ps.header.stamp=ps.header.stamp;
  dbg_pose_pub_.publish(sensor_ps);
#endif

  //HACK
  // Extract the ElbowJRotate joint angle from the rotation of the tracked link around the
  //shadowhand_lowerarm frame. This is an approximation if the actual shadowarm_lowerarm link is not
  //in a pose corresponding to the one commanded by this node

  // tf::StampedTransform L_T_B; //pose of the base expressed in the loweram frame
  // lock_.lock();

  // try
  //   {
  //     //shamelessly hardcoded transform between the reference link and the base
  //     tf_list_.lookupTransform("/sr_arm/position/shadowarm_lowerarm",base_frame_id_,ros::Time(0) ,L_T_B);
  //   }
  // catch (tf::TransformException ex)
  //   {
  //   ROS_ERROR("%s",ex.what());
  //   }
  
  // lock_.unlock();

  // tf::Transform L_T_T=L_T_B*B_T_T; //pose of the tracked link expressed in the shadowhand_lowerarm frame
  // double pitch,roll;

  
  // sr_robot_msgs::joint j;
  // j.joint_name="ElbowJRotate";
  // L_T_T.getBasis().getEulerYPR(j.joint_target,pitch, roll);//get the  ElbowJRoate joint angle (around the z-axis of the shadowhand_lowerarm frame)

  // std::cout<<"yaw: "<<j.joint_target*(-180)/PI<<" pitch: "<<pitch*(-180)/PI<<" roll: "<<roll*(-180)/PI<<std::endl;

  // j.joint_target=j.joint_target*(-180)/PI;

  // sr_robot_msgs::sendupdate msg;
  // msg.sendupdate_list.push_back(j);
  // msg.sendupdate_length=1;
  
  // EJR_pub_.publish(msg);
  //HACK END

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

  tf::VectorTFToEigen(B_T_T.getOrigin(),tl_init_pos_); //memorize the initial position of the tracked link - necessary for the safety zone maintainence
  B_T_E_=B_T_T*T_T_S_*E_T_S.inverse(); //setting the static transformation from the emitter to the base_link coordinate frame

  lock_.unlock();

  //Subscribe to the sensor data publisher - the remapping from sensor poses to tracked link poses is done in the sensorCallback
    sensor_poses_sub_ = nh_.subscribe("pose", 2, &SrArmTeleop::sensorCallback, this); 

  //HACK 
    //subscribe to the sendupdate publisher which will only publish joint angles for ElbowJRotate
    //EJR_pub_ = nh_.advertise<sr_robot_msgs::sendupdate>("sendupdate",1);
 //HACK END
  return true;
}
//-------------------------------------------------
bool SrArmTeleop::stopTeleop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
 
  //stop remapping the poses from the sensor to the controller
   sensor_poses_sub_.shutdown();

 //HACK 
   //shutdown to sendupdate publisher 
   //   EJR_pub_.shutdown();
 //HACK END
  
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
