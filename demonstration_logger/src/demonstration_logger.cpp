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
#include "rosbag/bag.h"
#include <sr_robot_msgs/joint.h>
#include <std_msgs/Float64.h>
// #include <sys/time.h>
// #include <time.h>
//#include <XmlRpcValue.h>
#include <vector>

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
    take_snapshot_srv_= nh_.advertiseService("take_snapshot",&DemonstrationLogger::takeSnapshot,this);
      
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

  joint_map_["THJ1"]=0;
  joint_map_["THJ2"]=0;
  joint_map_["THJ3"]=0;
  joint_map_["THJ4"]=0;
  joint_map_["THJ5"]=0;
  joint_map_["FFJ0"]=0;
  joint_map_["FFJ3"]=0;
  joint_map_["FFJ4"]=0;
  joint_map_["MFJ0"]=0;
  joint_map_["MFJ3"]=0;
  joint_map_["MFJ4"]=0;
  joint_map_["RFJ0"]=0;
  joint_map_["RFJ3"]=0;
  joint_map_["RFJ4"]=0;
  joint_map_["LFJ0"]=0;
  joint_map_["LFJ3"]=0;
  joint_map_["LFJ4"]=0;
  joint_map_["LFJ5"]=0;
  joint_map_["WRJ1"]=0;
  joint_map_["WRJ2"]=0;
  joint_map_["ElbowJRotate"]=0;
  joint_map_["ElbowJSwing"]=0;
  joint_map_["ShoulderJRotate"]=0;
  joint_map_["ShoulderJSwing"]=0;

}
//-------------------------------------------------------------------
void DemonstrationLogger::handJointStatesCallback(sr_robot_msgs::sendupdate::ConstPtr msg,std::string log_name)
{
  Eigen::VectorXd joint_states;
  sendupdateToEigen(msg,joint_states);
  std_msgs::Float64 angle;

  lock_.lock();
  std::ofstream file;
  rosbag::Bag bag;
  bag.open((log_dir_+log_name+".bag").c_str(), rosbag::bagmode::Append);
  file.open((log_dir_+log_name+".txt").c_str(), std::ios::out | std::ios::app );

  file<<ros::Time::now().toNSec()<<" ";
  for( int i=0; i<joint_states.size();i++)
    {
      file<<joint_states(i)<<" ";
      angle.data=joint_states(i)*PI/180;
      bag.write(msg->sendupdate_list[i].joint_name+"_topic", ros::Time::now(), angle);
    }

  file<<"\n";

  file.close();
  bag.close();

  lock_.unlock();

}
//-------------------------------------------------------------------
void DemonstrationLogger::snapshotCallback(sensor_msgs::JointState::ConstPtr msg,std::string log_name)
{
  //jointStateToEigen(msg,joint_states);  
  unsigned int n_joints=msg->name.size();
  // sr_robot_msgs::sendupdate sud;
  // sr_robot_msgs::joint joint;
  //sud.sendupdate_length=n_joints;
  
  lock_.lock();
  initHandJoints();

  std::map<std::string,double>::iterator it;
//std::string name;

for (unsigned int i=0; i<n_joints;i++)
  {
    it=joint_map_.find(msg->name[i].c_str());

    if(it!=joint_map_.end())
      {
	(*it).second=msg->position[i];
      }
    else if((std::strcmp(msg->name[i].c_str(),"FFJ1") == 0) || (std::strcmp(msg->name[i].c_str(),"FFJ2") == 0))
      {
          it=joint_map_.find("FFJ0");
          (*it).second=(*it).second+msg->position[i];
      }
    else if((std::strcmp(msg->name[i].c_str(),"MFJ1") == 0) || (std::strcmp(msg->name[i].c_str(),"MFJ2") == 0))
      {
          it=joint_map_.find("MFJ0");
          (*it).second=(*it).second+msg->position[i];
      }
    else if((std::strcmp(msg->name[i].c_str(),"RFJ1") == 0) || (std::strcmp(msg->name[i].c_str(),"RFJ2") == 0))
      {
          it=joint_map_.find("RFJ0");
          (*it).second=(*it).second+msg->position[i];
      }
    else if((std::strcmp(msg->name[i].c_str(),"LFJ1") == 0) || (std::strcmp(msg->name[i].c_str(),"LFJ2") == 0))
      {
          it=joint_map_.find("LFJ0");
          (*it).second=(*it).second+msg->position[i];
      }
}


   std::ofstream file;
  file.open((log_dir_+log_name+".txt").c_str(), std::ios::out | std::ios::app );

  file<<"#time";

  for (it=joint_map_.begin(); it !=joint_map_.end(); ++it)
    file<<"#"+it->first;

  // for (unsigned int i=0; i<n_joints;i++)
  //   {
  //     bool has_j2=false;
  //     if ( (std::strcmp(msg->name[i].c_str(),"THJ1") == 0) || (std::strcmp(msg->name[i].c_str(),"THJ2") == 0) || (std::strcmp(msg->name[i].c_str(),"WRJ1") == 0) || (std::strcmp(msg->name[i].c_str(),"WRJ2") == 0))
  // 	has_j2=true;

  //  if ( ((std::strcmp((msg->name[i].substr(3,1)).c_str(),"1")==0 || std::strcmp((msg->name[i].substr(3,1)).c_str(),"2")==0) && !has_j2) || (std::strcmp(msg->name[i].c_str(),"arm_link")==0))
  // 	continue;
 

  //   file<<"#"+msg->name[i];
  //   }

  file<<'\n';  

  // file<<ros::Time::now().toNSec()<<" ";
  // for(unsigned int i=0; i<n_joints;i++)
  //   {
  //   file<<msg->position[i]*180/PI<<" ";
  //   joint.joint_name=msg->name[i];
  //   joint.joint_target=msg->position[i]*180/PI;
  //   sud.sendupdate_list.push_back(joint);
  //   }

  std_msgs::Float64 angle;
  rosbag::Bag bag;
  bag.open((log_dir_+log_name+".bag").c_str(), rosbag::bagmode::Append);
  file<<ros::Time::now().toNSec()<<" ";

  for (it=joint_map_.begin(); it !=joint_map_.end(); ++it)
    {
    file<<it->second<<" ";
    angle.data=it->second;
     bag.write(it->first+"_topic", ros::Time::now(), angle);
    }

  // for(unsigned int i=0; i<n_joints;i++)
  //   {
  //     bool has_j2=false;
  //     if ( (std::strcmp(msg->name[i].c_str(),"THJ1") == 0) || (std::strcmp(msg->name[i].c_str(),"THJ2") == 0) || (std::strcmp(msg->name[i].c_str(),"WRJ1") == 0) || (std::strcmp(msg->name[i].c_str(),"WRJ2") == 0))
  // 	has_j2=true;

  //     if ( ((std::strcmp((msg->name[i].substr(3,1)).c_str(),"1")==0 || std::strcmp((msg->name[i].substr(3,1)).c_str(),"2")==0) && !has_j2) || (std::strcmp(msg->name[i].c_str(),"arm_link")==0))
  // 	continue;

  //     file<<msg->position[i]<<" ";
  //     angle.data=msg->position[i];
  //     bag.write(msg->name[i]+"_topic", ros::Time::now(), angle);
  //   }

  file<<"\n";

  file.close();
  bag.close();

  lock_.unlock();

  snapshot_sub_.shutdown();
  ROS_INFO("Wrote Snapshot to %s",(log_dir_+log_name+".txt").c_str());
}
//-------------------------------------------------------------------
// void DemonstrationLogger::handJointStatesGazeboCallback(sensor_msgs::JointState::ConstPtr msg,std::string log_name)
// {
//   //  struct timeval start, end;
//   // double c_time;
//   // gettimeofday(&start,0);
 
//    Eigen::VectorXd joint_states;
//    gazeboJointStatesToEigen(msg,joint_states);

//    std::ofstream file;
//    file.open((log_dir_+log_name+"_gazebo.txt").c_str(), std::ios::out | std::ios::app );

//    file<<msg->header.stamp.toNSec()<<" ";
//    for(unsigned int i=0; i<joint_states.size();i++)
//     file<<joint_states(i)<<" ";

//   file<<"\n";


//   file.close();

//   // gettimeofday(&end,0);
//   // c_time = end.tv_sec - start.tv_sec + 0.000001 * (end.tv_usec - start.tv_usec);
//   // std::cout<<"Frequency: "<<1/c_time<<" Hz"<<std::endl;

// }
 //-------------------------------------------------------------------
void DemonstrationLogger::writeFieldNames(std::string const & file_path)
 {
   std::vector<std::string> joint_names(hand_joints_.size());
   for ( XmlRpc::XmlRpcValue::iterator it=hand_joints_.begin() ; it != hand_joints_.end(); it++ )
     joint_names[(int)it->second]=(std::string)it->first;

  std::fstream file;
  file.open(file_path.c_str(),std::ios::out);
  file<<"#time";

  for(unsigned int i=0;i<joint_names.size();i++)     
    file<<"#"+joint_names[i];

  file<<'\n';
   file.close();
}
 //-------------------------------------------------------------------
bool DemonstrationLogger::takeSnapshot(demonstration_logger::StartLog::Request &req, demonstration_logger::StartLog::Response &res)
 {
   if(req.log_name.empty())
     {
       ROS_ERROR("Invalid logging name, cannot start logging.");
       return false;
     }

   lock_.lock();

   std::fstream file;
   file.open((log_dir_+req.log_name+".txt").c_str(),std::ios::in);
   if(file.is_open())
   {
     file.close();
     ROS_ERROR("File %s already exists. Choose a different name.",(log_dir_+req.log_name+".txt").c_str());
     lock_.unlock();
     return false;
   }
 file.close();


   file.open((log_dir_+req.log_name+".bag").c_str(), std::ios::binary | std::ios::in);
   if(file.is_open())
   {
     file.close();
     ROS_ERROR("File %s already exists. Choose a different name.",(log_dir_+req.log_name+".bag").c_str());
     lock_.unlock();
     return false;
   }
   file.close();

   snapshot_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, boost::bind(&DemonstrationLogger::snapshotCallback,this,_1, req.log_name));

   //create an empty .bag file which will be appended in the JointStates callback
  rosbag::Bag bag;
  bag.open((log_dir_+req.log_name+".bag").c_str(), rosbag::bagmode::Write);
  bag.close();

   lock_.unlock();

   ROS_INFO("Saving snapshot to %s",(log_dir_+req.log_name+".txt").c_str());
   return true;
}
 //-------------------------------------------------------------------
bool DemonstrationLogger::startLog(demonstration_logger::StartLog::Request &req, demonstration_logger::StartLog::Response &res)
 {
   if(req.log_name.empty())
     {
       ROS_ERROR("Invalid logging name, cannot start logging.");
       return false;
     }

   lock_.lock();
   std::fstream file;
   file.open((log_dir_+req.log_name+".txt").c_str(),std::ios::in);
   if(file.is_open())
   {
     file.close();
     ROS_ERROR("File %s already exists. Choose a different name.",(log_dir_+req.log_name+".txt").c_str());
     lock_.unlock();
     return false;
   }
 file.close();

 writeFieldNames(log_dir_+req.log_name+".txt");

   // file.open((log_dir_+req.log_name+"_gazebo.txt").c_str(),std::ios::in);
   // if(file.is_open())
   // {
   //   file.close();
   //   ROS_ERROR("File %s already exists. Choose a different name.",(log_dir_+req.log_name+"_gazebo.txt").c_str());
   //   return false;
   // }
   // file.close();


   file.open((log_dir_+req.log_name+".bag").c_str(), std::ios::binary | std::ios::in);
   if(file.is_open())
   {
     file.close();
     ROS_ERROR("File %s already exists. Choose a different name.",(log_dir_+req.log_name+".bag").c_str());
     lock_.unlock();
     return false;
   }
   file.close();

   hand_jointstates_sub_ = nh_.subscribe<sr_robot_msgs::sendupdate>("hand_joint_states", 10, boost::bind(&DemonstrationLogger::handJointStatesCallback,this,_1, req.log_name));
   // hand_jointstates_gazebo_sub_ = nh_.subscribe<sensor_msgs::JointState>("hand_joint_states_gazebo", 10, boost::bind(&DemonstrationLogger::handJointStatesGazeboCallback,this,_1, req.log_name));
  
   //create an empty .bag file which will be appended in the handJointStates callback
  rosbag::Bag bag;
  bag.open((log_dir_+req.log_name+".bag").c_str(), rosbag::bagmode::Write);
  bag.close();

 lock_.unlock();

   ROS_INFO("Started logging to %s",(log_dir_+req.log_name+".txt").c_str());
   return true;
}
 //-------------------------------------------------------------------
bool DemonstrationLogger::stopLog(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    lock_.lock();
    hand_jointstates_sub_.shutdown();
    //   hand_jointstates_gazebo_sub_.shutdown();
    lock_.unlock();
    ROS_INFO("Stopped logging");
    return true;
}


 //-------------------------------------------------------------------
void DemonstrationLogger::sendupdateToEigen(sr_robot_msgs::sendupdate::ConstPtr msg,Eigen::VectorXd & joint_states)
 {
   joint_states.resize(hand_joints_.size());
  
   for(unsigned int i=0; i<msg->sendupdate_list.size() ;i++)
     {
       if(hand_joints_.hasMember(msg->sendupdate_list[i].joint_name))
       joint_states((int)hand_joints_[msg->sendupdate_list[i].joint_name])=msg->sendupdate_list[i].joint_target;
     }
}
 //-------------------------------------------------------------------
// void DemonstrationLogger::jointStateToEigen(sensor_msgs::JointState::ConstPtr msg,Eigen::VectorXd & joint_states)
//  {
//    joint_states.resize(msg->name.size());
  
//    for(unsigned int i=0; i< joint_states.size() ;i++)
//      joint_states(i)=msg->position[i];
     
// }
//------------------------------------------------------------------------------------------------------
// void DemonstrationLogger::gazeboJointStatesToEigen(sensor_msgs::JointState::ConstPtr msg,Eigen::VectorXd & joint_states)
// {
//  joint_states.resize(hand_joints_.size());
//  std::string joint_name;
 
//  for(unsigned int i=0; i<msg->position.size();i++)
//      {
//        joint_name=msg->name[i];

//        if(!strcmp(joint_name.c_str(),"FFJ1"))
// 	 joint_name="FFJ0";
//        else if(!strcmp(joint_name.c_str(),"MFJ1"))
// 	 joint_name="MFJ0";
//        else if(!strcmp(joint_name.c_str(),"RFJ1"))
// 	 joint_name="RFJ0";
//        else if(!strcmp(joint_name.c_str(),"LFJ1"))
// 	 joint_name="LFJ0";

//        if(hand_joints_.hasMember(joint_name))
// 	 joint_states((int)hand_joints_[joint_name])=msg->position[i];
//      }
// }
//------------------------------------------------------------------------------------------------------

