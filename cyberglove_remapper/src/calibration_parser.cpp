
#include <ros/ros.h>

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include "calibration_parser.h"
#include <sstream>

namespace shadowhand_to_cyberglove_remapper {

const std::string CalibrationParser::default_abg_path_ = "../param/abg.txt";
const std::string CalibrationParser::default_gains_path_ = "../param/gains.txt";

CalibrationParser::CalibrationParser()
{
  ROS_WARN("No calibration path was specified, using default path");
  init(default_abg_path_,default_gains_path_);
}

CalibrationParser::CalibrationParser( std::string abg_path,std::string gains_path )
{
  init(abg_path,gains_path);
}

int CalibrationParser::init( std::string abg_path,std::string gains_path )
{
  //reserve enough lines
  abg_matrix_.reserve(25);
  gains_.reserve(25);
  std::ifstream abg_file;
  std::ifstream gains_file;
  abg_file.open(abg_path.c_str());
  gains_file.open(gains_path.c_str());
  //can't find the files
  if( !abg_file.is_open() )
    {
      ROS_ERROR("Couldn't open the file %s", abg_path.c_str());
      return -1;
    }
  if( !gains_file.is_open() )
    {
      ROS_ERROR("Couldn't open the file %s", gains_path.c_str());
      return -1;
    }

  std::string line;
  while( !abg_file.eof() )
    {
      getline(abg_file, line);

      //remove leading and trailing whitespaces
      line = boost::algorithm::trim_copy(line);

      //ignore empty line
      if( line.size() == 0 )
	continue;

      //ignore comments
      if( line[0] == '#' )
	continue;
     
      std::vector<std::string> splitted_string;
      boost::split(splitted_string, line, boost::is_any_of("\t "));
      splitted_string.erase( std::remove_if(splitted_string.begin(), splitted_string.end(), boost::bind( &std::string::empty, _1 ) ), splitted_string.end() );

      std::vector<double> double_line(splitted_string.size());
      for( unsigned int index_col = 0; index_col < splitted_string.size(); ++index_col )
	double_line[index_col] = convertToDouble(splitted_string[index_col]);

      abg_matrix_.push_back(double_line);
    }
  abg_file.close();

  while( !gains_file.eof() )
    {
      getline(gains_file, line);

      //remove leading and trailing whitespaces
      line = boost::algorithm::trim_copy(line);

      //ignore empty line
      if( line.size() == 0 )
	continue;

      //ignore comments
      if( line[0] == '#' )
	continue;
  
      std::vector<std::string> splitted_string;
      boost::split(splitted_string, line, boost::is_any_of("#"));
      splitted_string.erase( std::remove_if(splitted_string.begin(), splitted_string.end(), boost::bind( &std::string::empty, _1 ) ), splitted_string.end() );

      std::vector<double> double_line(splitted_string.size());
      for( unsigned int index_col = 0; index_col < splitted_string.size(); ++index_col )
	gains_.push_back(convertToDouble(splitted_string[index_col]));

    }
  gains_file.close();

  return 0;
}
}//end namespace


