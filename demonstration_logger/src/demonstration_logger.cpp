/**
 * @file   sr_traj_server.cpp
 * @author Robert Krug
 * @date   Fri May 11, 2012
 *
 */

#include <ros/ros.h>
#include "demonstration_logger/demonstration_logger.h"
#include <iostream>
#include <fstream>
#include <boost/bind.hpp>

//-------------------------------------------------------------------
DemonstrationLogger::DemonstrationLogger() : nh_private_("~")
{
    std::string searched_param;
    if(!nh_private_.searchParam("log_dir", searched_param))
    {
      ROS_ERROR("No logging directory specified on the parameter server - cannot start the demonstration logger.");
      ROS_BREAK();
    }     
    nh_private_.param(searched_param, log_dir_, std::string());
    ROS_INFO("Logging directory set to: %s",log_dir_.c_str());   

    initHandJoints();

    start_log_srv_ = nh_.advertiseService("start_log",&DemonstrationLogger::startLog,this);
    stop_log_srv_ = nh_.advertiseService("stop_log",&DemonstrationLogger::stopLog,this);

}
//-------------------------------------------------------------------
void DemonstrationLogger::initHandJoints()
{
  hand_joints_["THJ1"]=0;
  hand_joints_["THJ2"]=1;
  hand_joints_["THJ3"]=2;
  hand_joints_["THJ4"]=3;
  hand_joints_["THJ5"]=4;
  hand_joints_["FFJ0"]=5;
  hand_joints_["FFJ3"]=6;
  hand_joints_["FFJ4"]=7;
  hand_joints_["MFJ0"]=8;
  hand_joints_["MFJ3"]=9;
  hand_joints_["MFJ4"]=10;
  hand_joints_["RFJ0"]=11;
  hand_joints_["RFJ3"]=12;
  hand_joints_["RFJ4"]=13;
  hand_joints_["LFJ0"]=14;
  hand_joints_["LFJ3"]=15;
  hand_joints_["LFJ4"]=16;
  hand_joints_["LFJ5"]=17;
  hand_joints_["WRJ1"]=18;
  hand_joints_["WRJ2"]=19;
}
//-------------------------------------------------------------------
void DemonstrationLogger::handJointStatesCallback(sr_robot_msgs::sendupdate::ConstPtr msg,std::string log_name)
{
 Eigen::VectorXd joint_states;
 sendupdateToEigen(msg,joint_states);
 std::cout<<log_name<<std::endl;

 std::ofstream file;
 file.open((log_dir_+log_name+".txt").c_str(), std::ios::out | std::ios::app );

 file<<ros::WallTime::now().toNSec()<<" ";
 for(unsigned int i=0; i<joint_states.size();i++)
   file<<joint_states(i)<<" ";

 file<<"\n";
 file.close();











}
 //-------------------------------------------------------------------
bool DemonstrationLogger::startLog(demonstration_logger::StartLog::Request &req, demonstration_logger::StartLog::Response &res)
 {
   if(req.log_name.empty())
     {
       ROS_ERROR("Invalid logging name, cannot start logging.");
       return false;
     }

   std::fstream file;
   file.open((log_dir_+req.log_name+".txt").c_str(),std::ios::in);
   if(file.is_open())
   {
     file.close();
     ROS_ERROR("File %s already exists. Choose a different name.",(log_dir_+req.log_name+".txt").c_str());
     return false;
   }
   file.close();

   lock_.lock();
   hand_jointstates_sub_ = nh_.subscribe<sr_robot_msgs::sendupdate>("hand_joint_states", 1, boost::bind(&DemonstrationLogger::handJointStatesCallback,this,_1, req.log_name));

   lock_.unlock();

   ROS_INFO("Started logging to %s",(log_dir_+req.log_name+".txt").c_str());
   return true;
}

 //-------------------------------------------------------------------
bool DemonstrationLogger::stopLog(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    lock_.lock();
    hand_jointstates_sub_.shutdown();
    lock_.unlock();
    ROS_INFO("Stopped logging");
    return true;
}


 //-------------------------------------------------------------------
void DemonstrationLogger::sendupdateToEigen(sr_robot_msgs::sendupdate::ConstPtr msg,Eigen::VectorXd & joint_states)
 {
   joint_states.resize(hand_joints_.size());
  
   for(int i=0; i<hand_joints_.size();i++)
       joint_states((int)hand_joints_[msg->sendupdate_list[i].joint_name])=msg->sendupdate_list[i].joint_target;

}
//------------------------------------------------------------------------------------------------------


