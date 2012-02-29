#include "wintracker_publisher.h"
#include "WTracker.h"
#include <ros/ros.h>
#include <stdlib.h>
#include <cmath>

WintrackerPublisher::WintrackerPublisher() : nh_("~"), frame_id_("/fixed"), hemisphere_("Up"), pos_filter_(NULL), ori_filter_(NULL)
{
  std::string prefix;
  std::string frame_id;
  std::string searched_param;

  if(nh_.searchParam("hemisphere_specification",searched_param))
    nh_.getParam(searched_param, hemisphere_);

  // searches for parameter with name containing 'wintracker_prefix' 
  nh_.searchParam("wintracker_prefix", searched_param); 
  nh_.getParam(searched_param, prefix);

 if (nh_.searchParam("wintracker_frame_id", searched_param)) 
   {
    nh_.getParam(searched_param, frame_id);
    frame_id_=frame_id;
   }

  std::string full_topic = prefix + "/pose";
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>(full_topic, 2);
  pose_srv_=nh_.advertiseService(prefix + "/get_pose",&WintrackerPublisher::getPose,this);

  //initialize filters
  pos_filter_=new std::list<Eigen::Vector3f>(10,Eigen::Vector3f(0,0,0));
  ori_filter_=new std::list<Eigen::Vector4f>(10,Eigen::Vector4f(0,0,0,0));

   ori_in_=new std::deque<Eigen::Vector4f>(6,Eigen::Vector4f(0,0,0,0));
   ori_out_=new std::deque<Eigen::Vector4f>(6,Eigen::Vector4f(0,0,0,0));

}
geometry_msgs::PoseStamped WintrackerPublisher::getFilteredTick()
{

  // Eigen::Vector3f ftd_pos(0,0,0);
//   Eigen::Vector4f ftd_ori(0,0,0,0);

//   //get the filtered positions/orientations via averaging over the elements contained in the filter lists
//   //this could be done more efficient by just adjusting considering the popped element
//   for (std::list<Eigen::Vector3f>::iterator it = pos_filter_->begin(); it != pos_filter_->end(); it++)
//       ftd_pos+=(*it);
  
//   for (std::list<Eigen::Vector4f>::iterator it = ori_filter_->begin(); it != ori_filter_->end(); it++)
//       ftd_ori+=(*it);

//   ftd_pos=ftd_pos/pos_filter_->size();
//   ftd_ori=ftd_ori/ori_filter_->size();

//   //read data from the WinTracker
//   data_mutex_.lock();
//   tick_wtracker();
//   data_mutex_.unlock();   

//   Eigen::Vector3f curr_pos((float)wtrackerSensors[0].x,(float)wtrackerSensors[0].y,(float)wtrackerSensors[0].z);
//   Eigen::Vector4f curr_ori((float)wtrackerSensors[0].qx,(float)wtrackerSensors[0].qy,(float)wtrackerSensors[0].qz,(float)wtrackerSensors[0].qw);

//   // std::cout<<"Pos norm: "<<(curr_pos-ftd_pos).norm()<<std::endl;
//   //std::cout<<"Ori norm: "<<(curr_ori.head(3)-ftd_ori.head(3)).norm()<<std::endl;
//  std::cout<<"Pos norm: "<<(curr_pos-ftd_pos).norm()<<std::endl;
// std::cout<<"Ori norm: "<<(curr_ori.head(3)-ftd_ori.head(3)).norm()<<std::endl;
//   if((curr_pos-ftd_pos).norm() < 0.05)
//    {
//      //  std::cout<<"Pos norm: "<<(curr_pos-ftd_pos).norm()<<std::endl;
//  pos_filter_->pop_back();
//  pos_filter_->push_front(Eigen::Vector3f((float)wtrackerSensors[0].x,(float)wtrackerSensors[0].y,(float)wtrackerSensors[0].z));
//    }

//   if((curr_ori.head(3)-ftd_ori.head(3)).norm()<50)
//   {
//     //std::cout<<"Ori norm: "<<(curr_ori.head(3)-ftd_ori.head(3)).norm()<<std::endl;
//   ori_filter_->pop_back();
//   ori_filter_->push_front(Eigen::Vector4f((float)wtrackerSensors[0].qx,(float)wtrackerSensors[0].qy,(float)wtrackerSensors[0].qz,(float)wtrackerSensors[0].qw));
//   }
  // geometry_msgs::PoseStamped ps;
  // ps.pose.position.x = ftd_pos(0);
  // ps.pose.position.y = ftd_pos(1);
  // ps.pose.position.z = ftd_pos(2);
  // ps.pose.orientation.x = ftd_ori(0);
  // ps.pose.orientation.y = ftd_ori(1);
  // ps.pose.orientation.z = ftd_ori(2);
  // ps.pose.orientation.w = ftd_ori(3);
  // ps.header.frame_id = frame_id_;


  data_mutex_.lock();
  tick_wtracker();
  data_mutex_.unlock();   

 Eigen::Vector4f curr_ori((float)wtrackerSensors[0].qx,(float)wtrackerSensors[0].qy,(float)wtrackerSensors[0].qz,(float)wtrackerSensors[0].qw);
  adjustSigns(curr_ori);

  //5th order Butterworth filter
 ori_in_->pop_front();
 ori_in_->push_back(curr_ori/BW_GAIN);
 ori_out_->pop_front();

 curr_ori=(*ori_in_)[0]+(*ori_in_)[5]+5*((*ori_in_)[1]+(*ori_in_)[4])+10*((*ori_in_)[2]+ (*ori_in_)[3])
   +BW_0*(*ori_out_)[0]+BW_1*(*ori_out_)[1]+BW_2*(*ori_out_)[2]+BW_3*(*ori_out_)[3]+BW_4*(*ori_out_)[4];

 ori_out_->push_back(curr_ori);
 


//10th order Butterworth filter
 // ori_in_->pop_front();
 // ori_in_->push_back(curr_ori/BW_GAIN);
 // ori_out_->pop_front();

 // curr_ori=(*ori_in_)[0]+(*ori_in_)[10]+10*((*ori_in_)[1]+(*ori_in_)[9])+45*((*ori_in_)[2]+ (*ori_in_)[8])+120*((*ori_in_)[3]+(*ori_in_)[7])+210*((*ori_in_)[4]+ (*ori_in_)[6])+252*(*ori_in_)[5];
 // +BW_0*(*ori_out_)[0]+BW_1*(*ori_out_)[1]+BW_2*(*ori_out_)[2]+BW_3*(*ori_out_)[3]+BW_4*(*ori_out_)[4]+BW_5*(*ori_out_)[5]+BW_6*(*ori_out_)[6]+BW_7*(*ori_out_)[7]+BW_8*(*ori_out_)[8]+BW_9*(*ori_out_)[9];

 // ori_out_->push_back(curr_ori);

 // for(unsigned int i=0; i < ori_out_->size()-1;i++)
 //   curr_ori+=(*ori_out_)[i];

 // curr_ori=curr_ori/ori_out_->size();

 //(*ori_out_)[ori_out_->size()-1]=curr_ori;



 geometry_msgs::PoseStamped ps;
 ps.pose.position.x =0;
 ps.pose.position.y = 0;
 ps.pose.position.z = 0;
  ps.pose.orientation.x = curr_ori(0);
  ps.pose.orientation.y = curr_ori(1);
  ps.pose.orientation.z = curr_ori(2);
  ps.pose.orientation.w = curr_ori(3);
  ps.header.frame_id = frame_id_;

  return ps;
}

WintrackerPublisher::~WintrackerPublisher()
{
  delete pos_filter_;
  delete ori_filter_;
}
bool WintrackerPublisher::getPose(wintracker::GetPose::Request  &req, wintracker::GetPose::Response &res)
{

  //Reads only the sensor on the first serial port - could be changed via publishing a pose array
  res.pose_stamped=getFilteredTick();

  res.success=true;
  return res.success;
}
void WintrackerPublisher::startWTracker() 
{

  if(initialize_wtracker() != 0) 
  {
    ROS_ERROR("Failed to initialize wtracker, exiting");
    exit(0);
  }

  if(hemisphere_=="Up")
    setUpHemisphere();
  else if(hemisphere_=="Front")
    setFrontHemisphere();
  else
    {
      ROS_WARN("Hemisphere specification '%s' is invalid. It has to be either 'Up' or 'Front'. Setting hemisphere to 'Up'.",hemisphere_.c_str());
      setUpHemisphere();
    }

}

void WintrackerPublisher::shutdownWTracker() 
{
  shutdown_wtracker();
}

bool WintrackerPublisher::spin() {
  while (ros::ok()) 
   {
    pub_.publish(getFilteredTick());
    ros::spinOnce();
  }
  return true ;
}

void WintrackerPublisher::adjustSigns(Eigen::Vector4f& ori)
{
  
  if((ori_out_->back()(0))*ori(0) < 0 && abs(ori_out_->back()(0)-ori(0)) > M_DQX )
    ori(0)=(-ori(0));
 if((ori_out_->back()(1))*ori(1) < 0 && abs(ori_out_->back()(1)-ori(1)) > M_DQY )
    ori(1)=(-ori(1));
 if((ori_out_->back()(2))*ori(2) < 0 && abs(ori_out_->back()(2)-ori(2)) > M_DQZ )
    ori(2)=(-ori(2));
 if((ori_out_->back()(3))*ori(3) < 0 && abs(ori_out_->back()(3)-ori(3)) > M_DQW )
    ori(3)=(-ori(3));
}
