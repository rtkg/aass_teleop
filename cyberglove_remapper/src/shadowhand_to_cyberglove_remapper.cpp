/**
 * @file   shadowhand_to_cybergrasp_remapper.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
 *
*
* Copyright 2011 Shadowrobotw Robot Company Ltd.
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
 * @brief This program remapps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 *
 */

//ROS include
#include <ros/ros.h>

//generic include
#include <string>

//own .h
#include "shadowhand_to_cyberglove_remapper.h"
#include <sr_robot_msgs/sendupdate.h>
#include <sr_robot_msgs/joint.h>
using namespace ros;

namespace shadowhand_to_cyberglove_remapper
{

const int ShadowhandToCybergloveRemapper::number_hand_joints_ = 20;

ShadowhandToCybergloveRemapper::ShadowhandToCybergloveRemapper() :
  n_tilde_("~"), eigenspace_parser_(new EigenspaceParser()), espace_projection_(false)
{
    joints_names_.resize(number_hand_joints_);
    ShadowhandToCybergloveRemapper::initNames();

    std::string param;
    std::string abg_path;
    std::string gains_path;
    n_tilde_.searchParam("mapping_abg_path", param);
    n_tilde_.param(param, abg_path, std::string());
    n_tilde_.searchParam("mapping_gains_path", param);
    n_tilde_.param(param, gains_path, std::string());

    calibration_parser_ = new CalibrationParser(abg_path,gains_path);

    ROS_INFO("Mapping files loaded for the Cyberglove: %s, %s", abg_path.c_str(),gains_path.c_str());

    std::string prefix;
    std::string searched_param;
    n_tilde_.searchParam("cyberglove_prefix", searched_param);
    n_tilde_.param(searched_param, prefix, std::string());

    std::string full_topic = prefix + "/raw/joint_states";

    cyberglove_jointstates_sub_ = node_.subscribe(full_topic, 10, &ShadowhandToCybergloveRemapper::jointStatesCallback, this);

    n_tilde_.searchParam("sendupdate_prefix", searched_param);
    n_tilde_.param(searched_param, prefix, std::string());
    full_topic = prefix + "sendupdate";

    shadowhand_pub_ = node_.advertise<sr_robot_msgs::sendupdate> (full_topic, 5);
}

ShadowhandToCybergloveRemapper::~ShadowhandToCybergloveRemapper()
{
  delete calibration_parser_;
  delete eigenspace_parser_;
}

void ShadowhandToCybergloveRemapper::initNames()
{
    joints_names_[0] = "THJ1";
    joints_names_[1] = "THJ2";
    joints_names_[2] = "THJ3";
    joints_names_[3] = "THJ4";
    joints_names_[4] = "THJ5";
    joints_names_[5] = "FFJ0";
    joints_names_[6] = "FFJ3";
    joints_names_[7] = "FFJ4";
    joints_names_[8] = "MFJ0";
    joints_names_[9] = "MFJ3";
    joints_names_[10] = "MFJ4";
    joints_names_[11] = "RFJ0";
    joints_names_[12] = "RFJ3";
    joints_names_[13] = "RFJ4";
    joints_names_[14] = "LFJ0";
    joints_names_[15] = "LFJ3";
    joints_names_[16] = "LFJ4";
    joints_names_[17] = "LFJ5";
    joints_names_[18] = "WRJ1";
    joints_names_[19] = "WRJ2";
}

double ShadowhandToCybergloveRemapper::linRegression(int ordinal, double sv1, double sv2)
{
  return (calibration_parser_->gains_[ordinal]*(calibration_parser_->abg_matrix_[ordinal][0]+sv1*calibration_parser_->abg_matrix_[ordinal][1]
         +sv2*calibration_parser_->abg_matrix_[ordinal][2]))*180/PI;
}
double ShadowhandToCybergloveRemapper::checkJointLimits(double angle,double lower,double upper)
{
  double new_angle;
  new_angle = (angle > upper) ? upper : angle;
  new_angle = (angle < lower) ? lower : angle;

  return new_angle;
}
double ShadowhandToCybergloveRemapper::linWristMapping(std::string const & joint,double const sv)
{
  double angle=0;
  if(joint=="G_WristPitch")
      angle=80/0.55*sv-35;
  else if(joint=="G_WristYaw")
      angle=10-40/0.9*sv;
  else
    ROS_ERROR("Invalid wrist joint specified in ShadowhandToCybergloveRemapper::linWristMapping(std::string const & joint,double const sv)");

  return angle;
}

std::vector<double> ShadowhandToCybergloveRemapper::getRemappedVector(std::vector<double> const & glove_values)
{
  std::vector<double> joint_angles(number_hand_joints_,0);

  //The following corresponds to the linear regression approach by UHAM; The alpha-beta-gamma values
  //are read from abg.txt, the gains are read from gains.txt. Both aforementioned files are
  //generated by the UHAM Java Calibration Routine which utilizes GraspIt using the
  //ShadowhandLast.xml model;

//  The joints of the actual platform (not used here)
//   0..90   distal joint / FFJ1 MFJ1 RFJ1 LFJ1  (underactuated from J2)
//   0..90   medial joint / FFJ2 MFJ2 RFJ2 LFJ2  (also controlling J1)
//   0..90   proximal joint / FFJ3 MFJ4 RFJ3 LFJ3
// -25..25   abduction joint FFJ4 MFJ4 RFJ4 LFJ4
//   0..40   palm-arch joint / LFJ5

// -10..90   thumb distal flex / THJ1
// -30..30   thumb medial flex / thJ2
// -15..15   thumb medial abduction / THJ3
//   0..75   thumb base flex THJ4
// -60..60   thumb base rotation THJ5

  joint_angles[0]=checkJointLimits(linRegression(17,glove_values[2],0),0,90); //ThumbPI_DIJ - THJ1
  joint_angles[1]=checkJointLimits(linRegression(16,glove_values[1],0),-30,30);//ThumbMPJ - THJ2
  joint_angles[2]=checkJointLimits(linRegression(15,glove_values[1],glove_values[3]),-15,15); //ThumbProxLat (coupled with ThumbAb) - THJ3
  joint_angles[3]=checkJointLimits(linRegression(14,glove_values[3],0),0,70); //ThumbAb - THJ4 (the model in GraspIt has actually a joint angle range 0 -75; this difference is ignored here)
  joint_angles[4]=checkJointLimits(linRegression(13,glove_values[0],0),-60,60); //ThumbRotate - THJ5
  joint_angles[5]=checkJointLimits(linRegression(12,(2*glove_values[5]+glove_values[6])/3,0)*2,0,180); //IndexPI-DIJ - FFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
  joint_angles[6]=checkJointLimits(linRegression(11,glove_values[4],0),0,90);//IndexMPJ - FFJ3
  joint_angles[7]=checkJointLimits(linRegression(10,glove_values[10],0),-5,5);//IndexAb - FFJ4; the same joint range as in GraspIt was chosen although FFJ4 can range from +/-25 degree
  joint_angles[8]=checkJointLimits(linRegression(9,(2*glove_values[8]+glove_values[9])/3,0)*2,0,180);//MiddlePI-DIJ -MFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
  joint_angles[9]=checkJointLimits(linRegression(8,glove_values[7],0),0,90);//MiddleMPJ - MFJ3
  joint_angles[10]=checkJointLimits(linRegression(7,glove_values[14],0),-5,5);//MiddleAb - MFJ4; the same joint range as in GraspIt was chosen although MFJ4 can range from +/-25 degree
  joint_angles[11]=checkJointLimits(linRegression(6,(2*glove_values[12]+glove_values[13])/3,0)*2,0,180);//RingPI-DIJ - RFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
  joint_angles[12]=checkJointLimits(linRegression(5,glove_values[11],0),0,90);//RingMPJ - RFJ3
  joint_angles[13]=-checkJointLimits(linRegression(4,glove_values[18],glove_values[14]),-5,5);//RingAb (coupled with PinkieAb) - RFJ4; the same joint range as in GraspIt was chosen although FFJ4 can range from +/-25 degree; signs need to be switched
  joint_angles[14]=checkJointLimits(linRegression(3,(2*glove_values[16]+glove_values[17])/3,0)*2,0,180);//PinkiePI-DIJ - LFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
  joint_angles[15]=checkJointLimits(linRegression(2,glove_values[15],0),0,90);//PinkieMPJ - LFJ3
  joint_angles[16]=-checkJointLimits(linRegression(1,glove_values[18],glove_values[14]),-10,25);//PinkieAb (coupled with MiddleAb) - LFJ4; the same joint range as in GraspIt was chosen although FFJ4 can range from +/-25 degree; signs need to be switched
  joint_angles[17]=checkJointLimits(linRegression(0,glove_values[19],0),0,40);//PinkiePalm - LFJ5

  //Hardcoded linear mapping for the wrist joints
  joint_angles[18]=checkJointLimits(linWristMapping("G_WristPitch",glove_values[20]),-35,45);
  joint_angles[19]=checkJointLimits(linWristMapping("G_WristYaw",glove_values[21]),-30,10);

  return joint_angles;
}
void ShadowhandToCybergloveRemapper::jointStatesCallback( const sensor_msgs::JointStateConstPtr& msg )
{
    sr_robot_msgs::joint joint;
    sr_robot_msgs::sendupdate pub;

    //Do conversion
    std::vector<double> vect = getRemappedVector(msg->position);
    //Generate sendupdate message
    pub.sendupdate_length = number_hand_joints_;

    std::vector<sr_robot_msgs::joint> table(number_hand_joints_);
    for(int i = 0; i < number_hand_joints_; ++i )
    {
        joint.joint_name = joints_names_[i];
        joint.joint_target = vect[i];
        table[i] = joint;
    }
    pub.sendupdate_length = number_hand_joints_;
    pub.sendupdate_list = table;
    shadowhand_pub_.publish(pub);
}
}//end namespace
