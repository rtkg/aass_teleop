/**
* @file   eigenspace_parser.h
* @author Robert Krug <robert.krug@oru.se>
* @date   Thu, Feb 16, 2012
*
*
* @brief Loads and parses taxonomy based eigenspaces 
*
*
*/
//Modified 2012/02/14 by Robert Krug to incorporate the UHAM mapping via linear regression


#ifndef   	EIGENSPACE_PARSER_H_
# define   	EIGENSPACE_PARSER_H_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Core>

namespace shadowhand_to_cyberglove_remapper {

/**
* This is where the linear regression parameters are read and parsed
*/
class EigenspaceParser{

public:

   friend class ShadowhandToCybergloveRemapper;

  /**
  * Default constructor, using the default path. Initialize an
  * eigenspace matrix for the global eigenspace
  *
  */
  EigenspaceParser();
  ~EigenspaceParser();

private:

  Eigen::Matrix

  inline double convertToDouble(std::string const& s)
  {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
      ROS_ERROR("Bad calibration file: %s", s.c_str());
    return x;
  }
}; // end class

}//end namespace
#endif 	    /* !EIGENSPACE_PARSER_H_ */
