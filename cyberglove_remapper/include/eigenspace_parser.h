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

#ifndef   	EIGENSPACE_PARSER_H_
#define   	EIGENSPACE_PARSER_H_

#define MAX_DIM 50

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
  EigenspaceParser(std::string espace_dir);

  ~EigenspaceParser(){};

  void readEspace(std::string espace_type);

private:

/**
  * Flag indicating whether the eigenspace projection is active or not
  */
  bool espace_loaded_;
/**
  * Matrix holding the basis vector of the currently used eigenspace
  */
  Eigen::MatrixXd espace_;
/**
  * Vector with the mean values of the corresponding eigenspace
  */
  Eigen::VectorXd espace_offset_;

  std::string espace_dir_;
  
  inline double convertToDouble(std::string const& s)
  {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
      ROS_ERROR("Bad eigenspace file: %s", s.c_str());
    return x;
  }
}; // end class

}//end namespace
#endif 	    /* !EIGENSPACE_PARSER_H_ */
