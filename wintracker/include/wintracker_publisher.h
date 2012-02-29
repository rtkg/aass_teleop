/** \file wintracker_node.h
 * \author Krzysztof Charusta 
 *
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef WINTRACKER_PUBLISHER_H
#define WINTRACKER_PUBLISHER_H

#include <ros/ros.h>
#include <string.h>
#include "../srv_gen/cpp/include/wintracker/GetPose.h"
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include "geometry_msgs/PoseStamped.h"
#include <deque>

#define M_DQX 5
#define M_DQY 5
#define M_DQZ 5
#define M_DQW 5


//Coefficients for the Butterworth filter - sample rate: 80, cutoff frequency: 1,5
#define BW_GAIN 1.465027228e+14   //  1.465027228e+14
#define BW_0  -0.6051787989  //  -0.6051787989
#define BW_1 6.3556200682  //   6.3556200682
#define BW_2 -30.0441273930    //    -30.0441273930
#define BW_3 84.1848641440   //   84.1848641440
#define BW_4 -154.8445538800   //  -154.8445538800
#define BW_5 195.3533794700 // 195.3533794700
#define BW_6 -171.2005073300  //  -171.2005073300
#define BW_7  102.9095719200 //  102.9095719200
#define BW_8  -40.6070147940//  -40.6070147940
#define BW_9  9.4979465911 // 9.4979465911


/** \brief interface to wintracker 6d pose tracker
 *
 * Class is an interface to wintracker 6d pose. Publishes two topics:
 * Pose (pub_), and PoseStamped (pubTest_). PoseStamped has a
 * hardcoded header.frame_id = "/fixed" necessary for visualisation in
 * rviz. (such frame need to exist).
 *
 * Note: To run this publisher without sudo privilates you need to add
 * wintracker to udev. Details how to do that in manifest.
 * 
 */
class WintrackerPublisher {
  
 public:
  WintrackerPublisher();
  virtual ~WintrackerPublisher();  
  void startWTracker();
  void shutdownWTracker();
  void startStreaming();
  bool spin();
  
 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::ServiceServer pose_srv_;
  std::string frame_id_;
  boost::mutex data_mutex_;
  std::string hemisphere_;
  std::list<Eigen::Vector3f>* pos_filter_;
  std::list<Eigen::Vector4f>* ori_filter_;

  std::deque<Eigen::Vector4f>* ori_out_;
  std::deque<Eigen::Vector4f>* ori_in_;


/* Recurrence relation: of the Butterworth filter */
/* y[n] = (  1 * x[n-10]) */
/*      + ( 10 * x[n- 9]) */
/*      + ( 45 * x[n- 8]) */
/*      + (120 * x[n- 7]) */
/*      + (210 * x[n- 6]) */
/*      + (252 * x[n- 5]) */
/*      + (210 * x[n- 4]) */
/*      + (120 * x[n- 3]) */
/*      + ( 45 * x[n- 2]) */
/*      + ( 10 * x[n- 1]) */
/*      + (  1 * x[n- 0]) */

/*      + ( BW_0 * y[n-10]) */
/*      + ( BW_1 * y[n- 9]) */
/*      + ( BW_2 * y[n- 8]) */
/*      + ( BW_3 * y[n- 7]) */
/*      + ( BW_4* y[n- 6]) */
/*      + ( BW_5 * y[n- 5]) */
/*      + ( BW_6*y[n- 4]) */
/*      + ( BW_7* y[n- 3]) */
/*      + ( BW_8 * y[n- 2]) */
/*      + ( BW_9 * y[n- 1]) */

  geometry_msgs::PoseStamped getFilteredTick(); 
  void adjustSigns(Eigen::Vector4f& ori);

  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool getPose(wintracker::GetPose::Request  &req, wintracker::GetPose::Response &res);

};

#endif
