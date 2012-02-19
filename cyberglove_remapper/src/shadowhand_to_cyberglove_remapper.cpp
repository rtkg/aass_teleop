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
  n_tilde_("~")
{
    proj_matrix_.setIdentity(number_hand_joints_,number_hand_joints_);

    joints_names_.resize(number_hand_joints_);
    ShadowhandToCybergloveRemapper::initNames();

    std::string param;
    std::string abg_path;
    std::string gains_path;
    std::string espace_dir;
    n_tilde_.searchParam("mapping_abg_path", param);
    n_tilde_.param(param, abg_path, std::string());
    n_tilde_.searchParam("mapping_gains_path", param);
    n_tilde_.param(param, gains_path, std::string());
    n_tilde_.searchParam("projection_espace_dir", param);
    n_tilde_.param(param, espace_dir, std::string());

    calibration_parser_ = new CalibrationParser(abg_path,gains_path);
    eigenspace_parser_ = new EigenspaceParser(espace_dir);

    ROS_INFO("Mapping files loaded for the Cyberglove: %s, %s", abg_path.c_str(),gains_path.c_str());
    ROS_INFO("Eigenspace projection directory set: %s", espace_dir.c_str());

    std::string prefix;
    std::string searched_param;
    n_tilde_.searchParam("cyberglove_prefix", searched_param);
    n_tilde_.param(searched_param, prefix, std::string());

    std::string full_topic = prefix + "/raw/joint_states";
    cyberglove_jointstates_sub_ = node_.subscribe(full_topic, 10, &ShadowhandToCybergloveRemapper::jointStatesCallback, this);

    n_tilde_.searchParam("remapper_prefix", searched_param);
    n_tilde_.param(searched_param, prefix, std::string());
    full_topic = prefix + "project_eigenspace";
    project_eigenspace_service_ = node_.advertiseService(full_topic,&ShadowhandToCybergloveRemapper::formProjMatrix,this);
   
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

void ShadowhandToCybergloveRemapper::projectOnEspace(Eigen::VectorXd & hand_joints)
{
  Eigen::VectorXd mean(number_hand_joints_); mean.setZero();
  mean.head(18)=eigenspace_parser_->espace_offset_;

  hand_joints=proj_matrix_*(hand_joints-mean)+mean;
}

  Eigen::VectorXd ShadowhandToCybergloveRemapper::getRemappedVector(std::vector<double> const & glove_values)
{
  Eigen::VectorXd joint_angles(number_hand_joints_);

  //The following corresponds to the linear regression approach by UHAM; The alpha-beta-gamma values
  //are read from abg.txt, the gains are read from gains.txt. Both aforementioned files are
  //generated by the UHAM Java Calibration Routine which utilizes GraspIt using the
  //ShadowhandLast.xml model;

//  The joints of the actual CR5 muscle platform 
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

  joint_angles[0]=linRegression(17,glove_values[2],0); //ThumbPI_DIJ - THJ1
  joint_angles[1]=linRegression(16,glove_values[1],0);//ThumbMPJ - THJ2
  joint_angles[2]=linRegression(15,glove_values[1],glove_values[3]); //ThumbProxLat (coupled with ThumbAb) - THJ3
  joint_angles[3]=linRegression(14,glove_values[3],0); //ThumbAb - THJ4 
  joint_angles[4]=linRegression(13,glove_values[0],0); //ThumbRotate - THJ5
  joint_angles[5]=linRegression(12,(2*glove_values[5]+glove_values[6])/3,0)*2; //IndexPI-DIJ - FFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
  joint_angles[6]=linRegression(11,glove_values[4],0);//IndexMPJ - FFJ3
  joint_angles[7]=linRegression(10,glove_values[10],0);//IndexAb - FFJ4; 
  joint_angles[8]=linRegression(9,(2*glove_values[8]+glove_values[9])/3,0)*2;//MiddlePI-DIJ -MFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
  joint_angles[9]=linRegression(8,glove_values[7],0);//MiddleMPJ - MFJ3
  joint_angles[10]=linRegression(7,glove_values[14],0);//MiddleAb - MFJ4; 
  joint_angles[11]=linRegression(6,(2*glove_values[12]+glove_values[13])/3,0)*2;//RingPI-DIJ - RFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
  joint_angles[12]=linRegression(5,glove_values[11],0);//RingMPJ - RFJ3
  joint_angles[13]=-linRegression(4,glove_values[18],glove_values[14]);//RingAb (coupled with PinkieAb) - RFJ4; signs need to be switched
  joint_angles[14]=linRegression(3,(2*glove_values[16]+glove_values[17])/3,0)*2;//PinkiePI-DIJ - LFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
  joint_angles[15]=linRegression(2,glove_values[15],0);//PinkieMPJ - LFJ3
  joint_angles[16]=-linRegression(1,glove_values[18],glove_values[14]);//PinkieAb (coupled with MiddleAb) - LFJ4; signs need to be switched;
  joint_angles[17]=linRegression(0,glove_values[19],0);//PinkiePalm - LFJ5
  joint_angles[18]=linWristMapping("G_WristPitch",glove_values[20]); //WRJ1
  joint_angles[19]=linWristMapping("G_WristYaw",glove_values[21]); //WRJ2

  projectOnEspace(joint_angles);
  
  //Check the joint limits - probably superfluous since they get checked in Shadow's code anyway
  joint_angles[0]=checkJointLimits(joint_angles[0],0,90); //THJ1
  joint_angles[1]=checkJointLimits(joint_angles[1],-30,30);//THJ2
  joint_angles[2]=checkJointLimits(joint_angles[2],-15,15); //THJ3
  joint_angles[3]=checkJointLimits(joint_angles[3],0,75); //THJ4
  joint_angles[4]=checkJointLimits(joint_angles[4],-60,60); //THJ5
  joint_angles[5]=checkJointLimits(joint_angles[5],0,180); //FFJ0
  joint_angles[6]=checkJointLimits(joint_angles[6],0,90);// FFJ3
  joint_angles[7]=checkJointLimits(joint_angles[7],-25,25);//FFJ4; range in GraspIt is +/-5
  joint_angles[8]=checkJointLimits(joint_angles[8],0,180);//MFJ0
  joint_angles[9]=checkJointLimits(joint_angles[9],0,90);//MFJ3
  joint_angles[10]=checkJointLimits(joint_angles[10],-25,25);//MFJ4; range in GraspIt is +/-5
  joint_angles[11]=checkJointLimits(joint_angles[11],0,180);//RFJ0
  joint_angles[12]=checkJointLimits(joint_angles[12],0,90);//RFJ3
  joint_angles[13]=checkJointLimits(joint_angles[13],-25,25);//RFJ4; range in GraspIt is +/-5
  joint_angles[14]=checkJointLimits(joint_angles[14],0,180);//LFJ0; 
  joint_angles[15]=checkJointLimits(joint_angles[15],0,90);//LFJ3
  joint_angles[16]=checkJointLimits(joint_angles[16],-25,25);//LFJ4; range in GraspIt is +10/-25
  joint_angles[17]=checkJointLimits(joint_angles[17],0,40);//LFJ5
  joint_angles[18]=checkJointLimits(joint_angles[18],-35,45);//WRJ1
  joint_angles[19]=checkJointLimits(joint_angles[19],-30,10);//WRJ2

  return joint_angles;
}
void ShadowhandToCybergloveRemapper::jointStatesCallback( const sensor_msgs::JointStateConstPtr& msg )
{
    sr_robot_msgs::joint joint;
    sr_robot_msgs::sendupdate pub;

    //Do conversion
    Eigen::VectorXd vect = getRemappedVector(msg->position);
    //Generate sendupdate message
    pub.sendupdate_length = number_hand_joints_;

    std::vector<sr_robot_msgs::joint> table(number_hand_joints_);
    for(int i = 0; i < number_hand_joints_; ++i )
    {
        joint.joint_name = joints_names_[i];
        joint.joint_target = vect(i);
        table[i] = joint;
    }
    pub.sendupdate_length = number_hand_joints_;
    pub.sendupdate_list = table;
    shadowhand_pub_.publish(pub);
}

  bool ShadowhandToCybergloveRemapper::formProjMatrix(cyberglove_remapper::project_eigenspace::Request  &req, cyberglove_remapper::project_eigenspace::Response &res)
 {
   res.success=false;
   if((req.dim < 1)|| (req.dim > 18))
     {
       ROS_ERROR("Invalid eigenspace dimension - the dimension has to be between 1 and 18 for the Shadow Hand");
       return res.success;
     }

   eigenspace_parser_->setEspace(req.type);
   if(!eigenspace_parser_->espace_set_)
     return res.success;
     
   //since an eigenspace constitutes an orthonormal basis, a projection matrix for solving the
   //corresponding least squares problem can be formed by simply computing P=E^T*E where E holds in the
   //rows the first dim components of the eigenspace. A joint_angle vector y (with removed mean) is projected onto the subspace via P*y.
   proj_matrix_.topLeftCorner(18,18)=eigenspace_parser_->espace_.topLeftCorner(req.dim,18).transpose()*eigenspace_parser_->espace_.topLeftCorner(req.dim,18);
   std::cout<<proj_matrix_.bottomRightCorner(5,5)<<std::endl;
   res.success=true;
   return res.success;
 }

}//end namespace
