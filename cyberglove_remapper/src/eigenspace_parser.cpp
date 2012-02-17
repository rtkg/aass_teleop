#include <ros/ros.h>

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include "eigenspace_parser.h"
#include <sstream>

namespace shadowhand_to_cyberglove_remapper {

  EigenspaceParser::EigenspaceParser() : espace_loaded_(false), espace_(MAX_DIM,MAX_DIM), espace_offset_(MAX_DIM){}
  
  void EigenspaceParser::loadEspace(std::string const espace_type)
  {

  }



}//end namespace


