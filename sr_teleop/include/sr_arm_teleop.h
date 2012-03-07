/**
 * @file   sr_arm_teleop.h
 * @author Robert Krug
 * @date   Tue, Mar 6, 2012
 *
 *
 * Publishes Cartesian Pose Controller commands to the arm controller. 
 *
 */

#ifndef sr_arm_teleop_h_
#define sr_arm_teleop_h_

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>
#include "tf/tf.h"
#include "../srv_gen/cpp/include/sr_teleop/MovePose.h"
#include <tf/transform_listener.h>

/**
 * Maps the poses obtained from a 6D pose sensor to poses of a tracked link of the robot in a given
 * base reference frame; To use, hold the sensor in an appropriate posture (e.g. hold your hand with
 * the sensor attached according to the wrist of the robot) and call the /start_teleop service. Subsequently, 
 * pose messages are sent to a cartesian pose tracking controller.  
 * 
 */

class SrArmTeleop
{
 public:

  SrArmTeleop();
  ~SrArmTeleop(){};

 private:

  ros::NodeHandle nh_, nh_private_;
  boost::mutex lock_;


/**
 * Static transformation from the emitter to the base coordinate frame
 */
  tf::Transform B_T_E_;
/**
 * Predefined static transformation from the sensor frame to the frame of the tracked link
 */
  tf::Transform T_T_S_;
 /**
 * Subscribes to the 6D poses S^T_E of the sensor in the emitter frame 
 */
  ros::Subscriber sensor_poses_sub_;
/**
 * Service to get the pose S^T_E of the sensor in the emitter frame - this is used in the synchronization step
 */
  ros::ServiceClient get_sensor_pose_clt_;

  ros::ServiceClient switch_ctrl_clt_;

#ifdef DEBUG
ros::Publisher sensor_pose_pub_;
#endif

  ros::Publisher pose_setpt_pub_;
  ros::ServiceServer start_teleop_srv_;
  ros::ServiceServer stop_teleop_srv_;

  std::string base_frame_id_;
  std::string track_frame_id_;
  std::vector<std::string> default_controllers_;
  std::string cart_pose_controller_;

  tf::TransformListener tf_list_;


  std::string getRelativeName(std::string & name);

  /////////////////
  //  CALLBACKS  //
  /////////////////

  void sensorCallback(const geometry_msgs::PoseStamped & ps); 
  bool startTeleop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool stopTeleop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  // bool movePose(sr_teleop::MovePose::Request &req, sr_teleop::MovePose::Response &res);

}; // end class

#endif
