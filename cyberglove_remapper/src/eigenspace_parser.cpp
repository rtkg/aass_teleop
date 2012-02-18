#include <ros/ros.h>

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include "eigenspace_parser.h"
#include <sstream>

namespace shadowhand_to_cyberglove_remapper {

  EigenspaceParser::EigenspaceParser() : espace_loaded_(false), espace_(MAX_DIM,MAX_DIM), 
                                         espace_offset_(MAX_DIM),espace_dir_("../param/")
  {
    ROS_WARN("No eigenspace directory was specified, using the default directory");
  }

  EigenspaceParser::EigenspaceParser(std::string espace_dir) : espace_loaded_(false), espace_(MAX_DIM,MAX_DIM), 
                                         espace_offset_(MAX_DIM),espace_dir_(espace_dir){}
  
  void EigenspaceParser::readEspace(std::string espace_type)
  {
    if(espace_type=="Global")

    else if(espace_type=="")

    else
      ROS_WARN("Invalid Eigenspace type");
  }



}//end namespace


