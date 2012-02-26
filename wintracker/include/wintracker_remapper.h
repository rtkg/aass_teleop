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

/**
 * Maps the poses obtained from the wintracker driver node to gazebo model states
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
  std::string gazebo_model_;
  std::string wintracker_prefix_;
  std::string gazebo_prefix_;
  tf::Transform remap_tf_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  void remapPose(const geometry_msgs::PoseStamped & ps); 
  bool startRemap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool stopRemap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

}; // end class

#endif
