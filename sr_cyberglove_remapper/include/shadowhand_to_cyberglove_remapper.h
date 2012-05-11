/**
 * @file   shadowhand_to_cyberglove_remapper.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
//Modified by Robert Krug 2012/02/15 - Implements the UHAM linear regression mapping and the option
//to project the generated joint angle vector onto a loaded eigenspace


#ifndef   	SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_
#define   	SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_

#define PI 3.141592653589

//messages
#include <sensor_msgs/JointState.h>
#include "calibration_parser.h"
#include "eigenspace_parser.h"
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>
#include "sr_cyberglove_remapper/project_eigenspace.h"

using namespace ros;

namespace shadowhand_to_cyberglove_remapper{

/**
 * This program uses linear regression to map the sensor values from a Cyberglove to joint states
 * for the Shadow Hand
 */
class ShadowhandToCybergloveRemapper
{
 public:
  /**
   * Init the publisher / subscriber, the joint names, read the calibratin matrix
   */
  ShadowhandToCybergloveRemapper();
  ~ShadowhandToCybergloveRemapper();
 private:
  /**
   * Number of joints in the hand
   */
  static const int number_hand_joints_;
  

  /**
   * Maps the values obtained via the linear regression
   *
   * @param glove_values the raw sensor readings from the glove
   */
  Eigen::VectorXd getRemappedVector(std::vector<double> const & glove_values);

  /**
   * Performs linear regression on the raw sensor readings from the glove
   *
   * @param ordinal - indexes the joints as ordered in ShadowhandToCybergloveRemapper::initNames()
   *        sv1, sv2 - raw sensor readings used for the regression
   */
  double linRegression(int ordinal,double sv1, double sv2);

 /**
   * Shamelessly hardcoded linear mapping for the two wrist joints
   *
   * @param angle - a string, either 'G_WristPitch' or 'G_WristYaw'
   *        sv - raw sensor reading from one of the according wrist angles
   */
  double linWristMapping(std::string const & angle,double const sv);
  //Cuts of the angle at the given bounds
  double checkJointLimits(double angle,double lower,double upper);
  /// ROS node handles
  NodeHandle node_, n_tilde_;
  /// Vector containing all the joints names for the shadowhand.
  std::vector<std::string> joints_names_; 
  /// subscriber to the jointstates topic from the cyberglove
  Subscriber cyberglove_jointstates_sub_;
  ///publish to the shadowhand sendupdate topic
 
  ros::V_Publisher shadowhand_pubs_;

  ///the calibration parser containing the mapping matrix
  ServiceServer project_eigenspace_service_;
  CalibrationParser* calibration_parser_;
  EigenspaceParser* eigenspace_parser_;
  void projectOnEspace(Eigen::VectorXd & hand_joints);
 
  Eigen::MatrixXd proj_matrix_;
  boost::mutex data_mutex_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  /**
   * process the joint_states callback: receives the message from the cyberglove node, remap it to the Dextrous hand and
   * publish this message on a given topic
   *
   * @param msg the joint_states message
   */
  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  bool formProjMatrix(sr_cyberglove_remapper::project_eigenspace::Request  &req, sr_cyberglove_remapper::project_eigenspace::Response &res);
  
}; // end class

} //end namespace

#endif 	    /* !SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_ */
