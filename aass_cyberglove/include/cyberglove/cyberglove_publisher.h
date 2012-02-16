/**
 * @file   cyberglove_publisher.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:25:55 2010
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
 * @brief  This class reads and publishes data concerning the
 * cyberglove. To publish those data, just call the publish()
 * function. 
 * 
 * 
 */

//Modified by Robert Krug, 14.02.2012 - Removed the Start/Calibration services, only the raw joint
//values are published


#ifndef   	CYBERGLOVE_PUBLISHER_H_
# define   	CYBERGLOVE_PUBLISHER_H_

#include <ros/ros.h>
#include <vector>1

//messages
#include <sensor_msgs/JointState.h>

using namespace ros;

namespace cyberglove_publisher{

class CyberglovePublisher
{
 public:
  /// Constructor
  CyberglovePublisher();
  
  /// Destructor
  ~CyberglovePublisher();

  void publish();
  bool isPublishing();
  void setPublishing(bool value);
 private:
  /////////////////
  //  CALLBACKS  //
  /////////////////

  //ros node handle
  NodeHandle node, n_tilde;
  Rate publish_rate;
  std::string path_to_glove;
  bool publishing;

  Publisher cyberglove_raw_pub;

  sensor_msgs::JointState jointstate_raw_msg;

  float* glovePositions;
  bool checkGloveState();

}; // end class CyberglovePublisher

} // end namespace
#endif 	    /* !CYBERGLOVE_PUBLISHER_H_ */
