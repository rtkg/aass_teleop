/**
 * @file   sr_arm_teleop.h
 * @author Robert Krug
 * @date   Tue, Mar 6, 2012
 *
 *@brief Publishes Cartesian Pose Controller commands to the arm controller. The pose commands are
 * recieved in form of geometry_msgs::StampedPose on the subscribed /pose topic. Right now, the
 * synchronization depends on a service provided by the wintracker node. It would be good to remove
 * this dependency to allow the use of different pose sensors.
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
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include "../srv_gen/cpp/include/sr_teleop/MovePose.h"
#include <sr_robot_msgs/sendupdate.h>

/**
 * @brief Maps the poses obtained from a 6D pose sensor to poses of a tracked link of the robot in a
 * given base reference frame; To use, hold the sensor in an appropriate posture (e.g. hold your
 * hand with the sensor attached according to the wrist of the robot) and call the /start_teleop
 * service. Subsequently, the default controllers are switched with a
 * robot_mechanism_controllers/CartesianPoseController which tracks the remapped sensor poses.
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
 *@brief Static transformation from the emitter to the base coordinate frame
 */
  tf::Transform B_T_E_;
/**
 * @brief Predefined static transformation from the sensor frame to the frame of the tracked link
 */
  tf::Transform T_T_S_;
 /**
 * @brief Subscribes to the 6D poses S^T_E of the sensor in the emitter frame 
 */
  ros::Subscriber sensor_poses_sub_;
/**
 * @brief Service to get the pose S^T_E of the sensor in the emitter frame - this is used in the synchronization step
 */
  ros::ServiceClient get_sensor_pose_clt_;
/**
 * @brief Client calling the switch command from the pr2_controller_manager
 */
  ros::ServiceClient switch_ctrl_clt_;
/**
 * @brief If the debug flag is set, the node will publish B_T_S, the pose of the sensor in the base coordinate frame
 */
#ifdef DEBUG
ros::Publisher sensor_pose_pub_;
#endif
/**
 * @brief Publishes the remapped poses as setpoints for the cartesian pose controller
 */
  ros::Publisher pose_setpt_pub_;
  ros::ServiceServer start_teleop_srv_;
  ros::ServiceServer stop_teleop_srv_;
  ros::ServiceServer set_home_srv_;
  ros::ServiceServer go_home_srv_;
  ros::Publisher hand_cfg_pub_;
  ros::Publisher arm_cfg_pub_;

  sr_robot_msgs::sendupdate arm_home_cfg_;
  sr_robot_msgs::sendupdate hand_home_cfg_;

/**
 * @brief Holds the id of the desired base frame. This has to conform to the root_link of the cartesian pose controller
 */
  std::string base_frame_id_;
/**
 * @brief Specifies the tracked link. This has to conform to the tip_link of the cartesian pose controller
 */
  std::string track_frame_id_;
/**
 * @brief Vector holding the names (according to the pr2_controller_manager) of the default controllers
 */
  std::vector<std::string> default_controllers_;
/**
 * @brief String holdin the name (according to the pr2_controller_manager) of the cartesian pose controller for the teleoperation mode
 */
  std::string cart_pose_controller_;

  tf::TransformListener tf_list_;
/**
 * @brief Radius of the spherical safety zone. In teleoperation mode, the origin of the tracked link
 * is forced to stay inside the safety zone which is centered at the initial position of the tracked
 * link when the teleoperation started (i.e. when the /start_teleop service was called)
 */
  double sz_rad_;
/**
 * @brief Vector holding the inital position of the tracked link expressed in the base frame -
 * necessary to maintian the safety zone
 */
  Eigen::Vector3d tl_init_pos_;
/**
 * @brief Helper function to get the relative name from a resolved name, e.g. argument name="/foo/bar" would return "bar"
 */ 
  std::string getRelativeName(std::string & name);

  /////////////////
  //  CALLBACKS  //
  /////////////////

/**
 * @brief Remaps poses from the sensor to poses of the tracked link. This function also acts as a
 * watchdog and freezes the setpoint positions if they leave the safety zone
 */ 
  void sensorCallback(const geometry_msgs::PoseStamped & ps); 
/**
 * @brief Switches from the default controllers to the CPC and computes the static transform B_T_E_
 * of the emitter in the base coordinate frame
 */ 
  bool startTeleop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
/**
 * @brief Switches from the cartesian pose controller to the default controllers
 */ 
  bool stopTeleop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool setHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);


}; // end class

#endif
