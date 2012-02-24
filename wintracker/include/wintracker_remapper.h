/**
 * @file   wintracker_remapper.h
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
 *
 *
 *Remaps the poses from the wintracker to the gazebo/set_model_state topic
 *
 */


#ifndef wintracker_remapper_h_
#define wintracker_remapper_h_

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>

//#include "../srv_gen/cpp/include/cyberglove_remapper/project_eigenspace.h"

/**
 * This program uses linear regression to map the sensor values from a Cyberglove to joint states
 * for the Shadow Hand
 */
class WinTrackerRemapper
{
 public:
  /**
   * Init the publisher / subscriber, the joint names, read the calibratin matrix
   */
  WinTrackerRemapper();
  ~WinTrackerRemapper(){};

 private:

  ros::NodeHandle nh_, nh_private_;
  boost::mutex data_mutex_;
  ros::Subscriber wintracker_poses_sub_;
  ros::Publisher model_state_pub_;
  std::string gazebo_model_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  void poseCallback(const geometry_msgs::PoseStamped & ps); 


 /*  Eigen::MatrixXd proj_matrix_; */
 /*  boost::mutex data_mutex_; */

 /*  /\** */
 /*   * process the joint_states callback: receives the message from the cyberglove node, remap it to the Dextrous hand and */
 /*   * publish this message on a given topic */
 /*   * */
 /*   * @param msg the joint_states message */
 /*   *\/ */
 /* 
 /*  bool formProjMatrix(cyberglove_remapper::project_eigenspace::Request  &req, cyberglove_remapper::project_eigenspace::Response &res); */
  
}; // end class

#endif
