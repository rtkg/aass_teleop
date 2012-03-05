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
#include <std_srvs/Empty.h>
#include "tf/tf.h"
#include <string>
#include <list>

/**
 * Maps the poses obtained from the wintracker driver node to gazebo model states; To use, hold the 
 * sensor in an appropriate posture (i.e. hold your hand with the sensor attached according to the model)
 * and call the /start_remap service
 * 
 * Todo: Right now, the relative transform between the sensor and the gazebo_model which is slaved
 * to sensor is hardcoded to fit the sensor on the back of the hand (+x_s towards fingers,-y_s
 * towards the palm) and the shadow_model (+z towards fingers, +x towards thumb). It would be nice
 * to give this pose offset via the parameter server and launch file
 */
class WinTrackerRemapper
{
 public:

  WinTrackerRemapper();
  ~WinTrackerRemapper(){};

 private:

  ros::NodeHandle nh_, nh_private_;
  boost::mutex data_mutex_;
  ros::Subscriber wintracker_poses_sub_;
  ros::Publisher model_state_pub_;
  ros::ServiceServer start_remap_srv_;
  ros::ServiceServer stop_remap_srv_;
  ros::ServiceClient gazebo_modstat_clt_;
  ros::ServiceClient wt_get_pose_clt_;
  ros::ServiceClient gazebo_unpause_clt_;
  ros::ServiceClient gazebo_pause_clt_;

  std::string gazebo_model_;
  std::string wintracker_prefix_;
  std::string gazebo_prefix_;
  // static transformation G^T_W from the wintracker to the Gazebo coordinate frame
  tf::Transform G_T_W_;
 //static transformation M^T_S from the sensor to the Gazebo model coordinate frame
  tf::Transform M_T_S_;
  /////////////////
  //  CALLBACKS  //
  /////////////////

  void remapPose(const geometry_msgs::PoseStamped & ps); 
  bool startRemap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool stopRemap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

}; // end class

#endif
