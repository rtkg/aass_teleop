/**
 * @file   demonstration_logger.h
 * @author Robert Krug
 * @date   Fri May 11, 2012
* 
* Node listening to the sendupdate hand joint state topic and writing the data to files
*/

#ifndef   	demonstration_logger_h_
#define   	demonstration_logger_h_

#define PI 3.14159265358

#include <Eigen/Core>
#include <boost/thread/mutex.hpp>
#include "demonstration_logger/StartLog.h"
#include "std_srvs/Empty.h"
#include <sr_robot_msgs/sendupdate.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <boost/noncopyable.hpp>

class DemonstrationLogger : public boost::noncopyable
{
  public: 
 
   DemonstrationLogger();

   // void spin();

  private: 

   void initHandJoints();
   void sendupdateToEigen(sr_robot_msgs::sendupdate::ConstPtr msg,Eigen::VectorXd & joint_states);
   // void gazeboJointStatesToEigen(sensor_msgs::JointState::ConstPtr msg,Eigen::VectorXd & joint_states);
   void writeFieldNames(std::string const & file_path);

  ros::NodeHandle nh_, nh_private_;
  XmlRpc::XmlRpcValue hand_joints_;
  
  boost::mutex lock_;
  std::string log_dir_;
  ros::ServiceServer start_log_srv_;
  ros::ServiceServer stop_log_srv_;
  ros::ServiceServer take_snapshot_srv_;
  ros::Subscriber hand_jointstates_sub_;
  ros::Subscriber snapshot_sub_;
  

 /*  ///////////////// */
 /*  //  CALLBACKS  // */
 /*  ///////////////// */
 
 void handJointStatesCallback(sr_robot_msgs::sendupdate::ConstPtr msg,std::string log_name);
 void snapshotCallback(sensor_msgs::JointState::ConstPtr msg,std::string log_name);
 // void handJointStatesGazeboCallback(sensor_msgs::JointState::ConstPtr msg,std::string log_name);
  bool startLog(demonstration_logger::StartLog::Request &req, demonstration_logger::StartLog::Response &res);
  bool stopLog(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool takeSnapshot(demonstration_logger::StartLog::Request &req, demonstration_logger::StartLog::Response &res);
}; // end class


#endif 	
