/**
 * @file   wintracker_remapper.cpp
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
 *
*
*/


//#include <string>

#include "wintracker_remapper.h"
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>

WinTrackerRemapper::WinTrackerRemapper() : nh_private_("~"), gazebo_model_("gplane")
{
  std::string searched_param;
  if(!nh_private_.searchParam("gazebo_model", searched_param))
    ROS_WARN("No Gazebo model specified - using %s as remapping reference", gazebo_model_.c_str());
  else
    nh_private_.getParam(searched_param, gazebo_model_);

  // searches for parameter with name containing 'wintracker_prefix' 
  nh_private_.searchParam("wintracker_prefix", searched_param); 
  nh_private_.getParam(searched_param, wintracker_prefix_);

  nh_private_.searchParam("gazebo_prefix", searched_param); 
  nh_private_.getParam(searched_param, gazebo_prefix_);


   start_remap_srv_ = nh_.advertiseService(wintracker_prefix_ + "/start_remap",&WinTrackerRemapper::startRemap,this);
   stop_remap_srv_ = nh_.advertiseService(wintracker_prefix_ + "/stop_remap",&WinTrackerRemapper::stopRemap,this);
   gazebo_modstat_clt_ =  nh_.serviceClient<gazebo_msgs::GetModelState>(gazebo_prefix_ + "/get_model_state");
  // 
  // 
 
    // n_private_.searchParam("remapper_prefix", searched_param);
    // n_private_.param(searched_param, prefix, std::string());
    // full_topic = prefix + "project_eigenspace";
    // project_eigenspace_service_ = node_.advertiseService(full_topic,&WinTrackerRemapper::formProjMatrix,this);
   

}
//---------------------------------------------------------------------------
// bool WinTrackerRemapper::getModelState(gazebo_msgs::ModelState::Request  &req, gazebo_msgs::ModelState::Response &res)
// {

// }
//---------------------------------------------------------------------------
bool WinTrackerRemapper::startRemap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  data_mutex_.lock();
  //get the initial pose of the model from Gazebo
  gazebo_msgs::GetModelState gaz_ms; 
  gaz_ms.request.model_name=gazebo_model_;
  gaz_ms.request.relative_entity_name="world";
  gazebo_modstat_clt_.call(gaz_ms);
  if(!gaz_ms.response.success)
    {
      ROS_ERROR("Could not get the state of model %s from gazebo. Cannot start remapping.",gazebo_model_.c_str());
      data_mutex_.unlock();
      return false;
    }

  tf::Transform gazebo_tf, wintrack_tf;
  
  gazebo_tf.setOrigin(tf::Vector3(gaz_ms.response.pose.position.x,gaz_ms.response.pose.position.y,gaz_ms.response.pose.position.z));
  gazebo_tf.setRotation(tf::Quaternion(gaz_ms.response.pose.orientation.x,gaz_ms.response.pose.orientation.y,gaz_ms.response.pose.orientation.z,gaz_ms.response.pose.orientation.w));

  //need another service in the wintracker_publisher

  remap_tf_.mult(gazebo_tf,wintrack_tf);//not sure about this



  

 model_state_pub_ = nh_.advertise<gazebo_msgs::ModelState> (gazebo_prefix_ + "/set_model_state", 2); 
 wintracker_poses_sub_ = nh_.subscribe(wintracker_prefix_ + "/pose", 2, &WinTrackerRemapper::poseRemap, this);

  data_mutex_.unlock();
  return true;
}
//---------------------------------------------------------------------------
bool WinTrackerRemapper::stopRemap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  data_mutex_.lock();
 
  wintracker_poses_sub_.shutdown();
  model_state_pub_.shutdown();  

  data_mutex_.unlock();
  return true;
}
//---------------------------------------------------------------------------
void WinTrackerRemapper::poseRemap(const geometry_msgs::PoseStamped & ps)
{
  gazebo_msgs::ModelState ms;
  
  ms.model_name=gazebo_model_;
  ms.pose=ps.pose;
  ms.reference_frame=ps.header.frame_id;

  model_state_pub_.publish(ms);
}
//---------------------------------------------------------------------------
// void WinTrackerRemapper::initNames()
// {
//     joints_names_[0] = "THJ1";
//     joints_names_[1] = "THJ2";
//     joints_names_[2] = "THJ3";
//     joints_names_[3] = "THJ4";
//     joints_names_[4] = "THJ5";
//     joints_names_[5] = "FFJ0";
//     joints_names_[6] = "FFJ3";
//     joints_names_[7] = "FFJ4";
//     joints_names_[8] = "MFJ0";
//     joints_names_[9] = "MFJ3";
//     joints_names_[10] = "MFJ4";
//     joints_names_[11] = "RFJ0";
//     joints_names_[12] = "RFJ3";
//     joints_names_[13] = "RFJ4";
//     joints_names_[14] = "LFJ0";
//     joints_names_[15] = "LFJ3";
//     joints_names_[16] = "LFJ4";
//     joints_names_[17] = "LFJ5";
//     joints_names_[18] = "WRJ1";
//     joints_names_[19] = "WRJ2";
// }

// double WinTrackerRemapper::linRegression(int ordinal, double sv1, double sv2)
// {
//   return (calibration_parser_->gains_[ordinal]*(calibration_parser_->abg_matrix_[ordinal][0]+sv1*calibration_parser_->abg_matrix_[ordinal][1]
//          +sv2*calibration_parser_->abg_matrix_[ordinal][2]))*180/PI;
// }
// double WinTrackerRemapper::checkJointLimits(double angle,double lower,double upper)
// {
//   double new_angle;
//   new_angle = (angle > upper) ? upper : angle;
//   new_angle = (angle < lower) ? lower : angle;

//   return new_angle;
// }
// double WinTrackerRemapper::linWristMapping(std::string const & joint,double const sv)
// {
//   double angle=0;
//   if(joint=="G_WristPitch")
//       angle=80/0.55*sv-35;
//   else if(joint=="G_WristYaw")
//       angle=10-40/0.9*sv;
//   else
//     ROS_ERROR("Invalid wrist joint specified in WinTrackerRemapper::linWristMapping(std::string const & joint,double const sv)");

//   return angle;
// }

// void WinTrackerRemapper::projectOnEspace(Eigen::VectorXd & hand_joints)
// {
//   Eigen::VectorXd mean(number_hand_joints_); mean.setZero();
//   mean.head(18)=eigenspace_parser_->espace_offset_;
//   //hand_joints=mean;
//    hand_joints=proj_matrix_*(hand_joints-mean)+mean;
// }

//   Eigen::VectorXd WinTrackerRemapper::getRemappedVector(std::vector<double> const & glove_values)
// {
//   Eigen::VectorXd joint_angles(number_hand_joints_);

//   //The following corresponds to the linear regression approach by UHAM; The alpha-beta-gamma values
//   //are read from abg.txt, the gains are read from gains.txt. Both aforementioned files are
//   //generated by the UHAM Java Calibration Routine which utilizes GraspIt using the
//   //ShadowhandLast.xml model;

// //  The joints of the actual CR5 muscle platform 
// //   0..90   distal joint / FFJ1 MFJ1 RFJ1 LFJ1  (underactuated from J2)
// //   0..90   medial joint / FFJ2 MFJ2 RFJ2 LFJ2  (also controlling J1)
// //   0..90   proximal joint / FFJ3 MFJ4 RFJ3 LFJ3
// // -25..25   abduction joint FFJ4 MFJ4 RFJ4 LFJ4
// //   0..40   palm-arch joint / LFJ5

// // -10..90   thumb distal flex / THJ1
// // -30..30   thumb medial flex / thJ2
// // -15..15   thumb medial abduction / THJ3
// //   0..75   thumb base flex THJ4
// // -60..60   thumb base rotation THJ5

//   joint_angles[0]=linRegression(17,glove_values[2],0); //ThumbPI_DIJ - THJ1
//   joint_angles[1]=linRegression(16,glove_values[1],0);//ThumbMPJ - THJ2
//   joint_angles[2]=linRegression(15,glove_values[1],glove_values[3]); //ThumbProxLat (coupled with ThumbAb) - THJ3
//   joint_angles[3]=linRegression(14,glove_values[3],0); //ThumbAb - THJ4 
//   joint_angles[4]=linRegression(13,glove_values[0],0); //ThumbRotate - THJ5
//   joint_angles[5]=linRegression(12,(2*glove_values[5]+glove_values[6])/3,0)*2; //IndexPI-DIJ - FFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
//   joint_angles[6]=linRegression(11,glove_values[4],0);//IndexMPJ - FFJ3
//   joint_angles[7]=linRegression(10,glove_values[10],0);//IndexAb - FFJ4; 
//   joint_angles[8]=linRegression(9,(2*glove_values[8]+glove_values[9])/3,0)*2;//MiddlePI-DIJ -MFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
//   joint_angles[9]=linRegression(8,glove_values[7],0);//MiddleMPJ - MFJ3
//   joint_angles[10]=linRegression(7,glove_values[14],0);//MiddleAb - MFJ4; 
//   joint_angles[11]=linRegression(6,(2*glove_values[12]+glove_values[13])/3,0)*2;//RingPI-DIJ - RFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
//   joint_angles[12]=linRegression(5,glove_values[11],0);//RingMPJ - RFJ3
//   joint_angles[13]=-linRegression(4,glove_values[18],glove_values[14]);//RingAb (coupled with PinkieAb) - RFJ4; signs need to be switched
//   joint_angles[14]=linRegression(3,(2*glove_values[16]+glove_values[17])/3,0)*2;//PinkiePI-DIJ - LFJ0; couple PI-DIJ sensor readings (factor 2 is necessary to get correct joint targets)
//   joint_angles[15]=linRegression(2,glove_values[15],0);//PinkieMPJ - LFJ3
//   joint_angles[16]=-linRegression(1,glove_values[18],glove_values[14]);//PinkieAb (coupled with MiddleAb) - LFJ4; signs need to be switched;
//   joint_angles[17]=linRegression(0,glove_values[19],0);//PinkiePalm - LFJ5
//   joint_angles[18]=linWristMapping("G_WristPitch",glove_values[20]); //WRJ1
//   joint_angles[19]=linWristMapping("G_WristYaw",glove_values[21]); //WRJ2

//   projectOnEspace(joint_angles);
  
//   //Check the joint limits - probably superfluous since they get checked in Shadow's code anyway
//   joint_angles[0]=checkJointLimits(joint_angles[0],0,90); //THJ1
//   joint_angles[1]=checkJointLimits(joint_angles[1],-30,30);//THJ2
//   joint_angles[2]=checkJointLimits(joint_angles[2],-15,15); //THJ3
//   joint_angles[3]=checkJointLimits(joint_angles[3],0,75); //THJ4
//   joint_angles[4]=checkJointLimits(joint_angles[4],-60,60); //THJ5
//   joint_angles[5]=checkJointLimits(joint_angles[5],0,180); //FFJ0
//   joint_angles[6]=checkJointLimits(joint_angles[6],0,90);// FFJ3
//   joint_angles[7]=checkJointLimits(joint_angles[7],-25,25);//FFJ4; range in GraspIt is +/-5
//   joint_angles[8]=checkJointLimits(joint_angles[8],0,180);//MFJ0
//   joint_angles[9]=checkJointLimits(joint_angles[9],0,90);//MFJ3
//   joint_angles[10]=checkJointLimits(joint_angles[10],-25,25);//MFJ4; range in GraspIt is +/-5
//   joint_angles[11]=checkJointLimits(joint_angles[11],0,180);//RFJ0
//   joint_angles[12]=checkJointLimits(joint_angles[12],0,90);//RFJ3
//   joint_angles[13]=checkJointLimits(joint_angles[13],-25,25);//RFJ4; range in GraspIt is +/-5
//   joint_angles[14]=checkJointLimits(joint_angles[14],0,180);//LFJ0; 
//   joint_angles[15]=checkJointLimits(joint_angles[15],0,90);//LFJ3
//   joint_angles[16]=checkJointLimits(joint_angles[16],-25,25);//LFJ4; range in GraspIt is +10/-25
//   joint_angles[17]=checkJointLimits(joint_angles[17],0,40);//LFJ5
//   joint_angles[18]=checkJointLimits(joint_angles[18],-35,45);//WRJ1
//   joint_angles[19]=checkJointLimits(joint_angles[19],-30,10);//WRJ2

//   return joint_angles;
// }
// void WinTrackerRemapper::jointStatesCallback( const sensor_msgs::JointStateConstPtr& msg )
// {
//     sr_robot_msgs::joint joint;
//     sr_robot_msgs::sendupdate pub;

//     //Do conversion
//     Eigen::VectorXd vect = getRemappedVector(msg->position);
//     //Generate sendupdate message
//     pub.sendupdate_length = number_hand_joints_;

//     std::vector<sr_robot_msgs::joint> table(number_hand_joints_);
//     for(int i = 0; i < number_hand_joints_; ++i )
//     {
//         joint.joint_name = joints_names_[i];
//         joint.joint_target = vect(i);
//         table[i] = joint;
//     }
//     pub.sendupdate_length = number_hand_joints_;
//     pub.sendupdate_list = table;
//     shadowhand_pub_.publish(pub);
// }

//   bool WinTrackerRemapper::formProjMatrix(cyberglove_remapper::project_eigenspace::Request  &req, cyberglove_remapper::project_eigenspace::Response &res)
//  {
//   data_mutex_.lock();

//    res.success=false;
//    if((req.dim < 1)|| (req.dim > 18))
//      {
//        ROS_ERROR("Invalid eigenspace dimension - the dimension has to be between 1 and 18 for the Shadow Hand");
//        data_mutex_.unlock();
//        return res.success;
//      }

//    eigenspace_parser_->setEspace(req.type);
//    if(!eigenspace_parser_->espace_set_)
//      {
//        data_mutex_.unlock();
//        return res.success;
//      }

//    if((eigenspace_parser_->espace_.cols() != 18)||(eigenspace_parser_->espace_.rows() != 18))
//    {
//        ROS_ERROR("The loaded eigenspace matrix has to be 18x18 for the Shadow Hand");
//        data_mutex_.unlock();
//        return res.success;
//    }
//    //since an eigenspace constitutes an orthonormal basis, a projection matrix for solving the
//    //corresponding least squares problem can be formed by simply computing P=E^T*E where E holds in the
//    //rows the first dim components of the eigenspace. A joint_angle vector y (with removed mean) is projected onto the subspace via P*y.
//    proj_matrix_.topLeftCorner(18,18)=eigenspace_parser_->espace_.topLeftCorner(req.dim,18).transpose()*eigenspace_parser_->espace_.topLeftCorner(req.dim,18);
  
//    res.success=true;
//    data_mutex_.unlock();

//    return res.success;
//  }


