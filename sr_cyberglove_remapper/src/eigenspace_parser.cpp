#include <ros/ros.h>

#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include "eigenspace_parser.h"
#include <sstream>

namespace shadowhand_to_cyberglove_remapper {

  EigenspaceParser::EigenspaceParser() : espace_set_(false), espace_dir_("../param/")
  {
    ROS_WARN("No eigenspace directory was specified, using the default directory");
    setEspace("Global");
  }

  EigenspaceParser::EigenspaceParser(std::string espace_dir) : espace_set_(false), espace_dir_(espace_dir)
  {
    setEspace("Global");
  }
  
  bool EigenspaceParser::parseFiles(std::string espace_path,std::string espace_offset_path)
  {
    //reserve enough lines
    espace_.resize(MAX_DIM,MAX_DIM);
    espace_offset_.resize(MAX_DIM);

    std::ifstream espace_file;
    std::ifstream espace_offset_file;
    espace_file.open(espace_path.c_str());
    espace_offset_file.open(espace_offset_path.c_str());
    //can't find the files
    if( !espace_file.is_open() )
      {
	ROS_ERROR("Couldn't open the file %s", espace_path.c_str());
	return false;
      }
    if( !espace_offset_file.is_open() )
      {
	ROS_ERROR("Couldn't open the file %s", espace_offset_path.c_str());
	return false;
      }

    unsigned int dim=0;
    std::string line;
    while( !espace_file.eof() )
      {
	getline(espace_file, line);

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
     
	for( unsigned int index_col = 0; index_col < splitted_string.size(); ++index_col )
	  espace_(dim,index_col) = convertToDouble(splitted_string[index_col]);

	dim+=1;
      }
    espace_file.close();
    espace_.conservativeResize(dim,dim); //Trim to a square matrix 

    while( !espace_offset_file.eof() )
      {
	getline(espace_offset_file, line);

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

	for( unsigned int index_col = 0; index_col < splitted_string.size(); ++index_col )
	  espace_offset_(index_col)= convertToDouble(splitted_string[index_col]);

      }
    espace_offset_file.close();
    espace_offset_.conservativeResize(dim);

    return true;
  }

  void EigenspaceParser::setEspace(std::string espace_type)
  {
    espace_set_=false;
   
    if(espace_type=="Global")
      {
	if(parseFiles(espace_dir_ + "GLOBALVECS.txt",espace_dir_ + "GLOBALMEAN.txt" )) espace_set_=true;
      }
    else if(espace_type=="Tripod")
      {
	if(parseFiles(espace_dir_ + "G1VECS_N.txt",espace_dir_ + "G1MEAN_N.txt" )) espace_set_=true;
      }
    else if(espace_type=="Palmar_Pinch")
      {
	if(parseFiles(espace_dir_ + "G2VECS_N.txt",espace_dir_ + "G2MEAN_N.txt" )) espace_set_=true;
      }
    else if(espace_type=="Lateral")
      {
	if(parseFiles(espace_dir_ + "G3VECS_N.txt",espace_dir_ + "G3MEAN_N.txt" )) espace_set_=true;
      }
    else if(espace_type=="Writing_Tripod")
      {
	if(parseFiles(espace_dir_ + "G4VECS_N.txt",espace_dir_ + "G4MEAN_N.txt" )) espace_set_=true;
      }
    else if(espace_type=="Parallel_Extension")
      {
	if(parseFiles(espace_dir_ + "G5VECS_N.txt",espace_dir_ + "G5MEAN_N.txt" )) espace_set_=true;
      }
    else if(espace_type=="Adduction_Grip")
      {
	if(parseFiles(espace_dir_ + "G6VECS_N.txt",espace_dir_ + "G6MEAN_N.txt" )) espace_set_=true;
      }
    else if(espace_type=="Tip_Pinch")
      {
	if(parseFiles(espace_dir_ + "G7VECS_N.txt",espace_dir_ + "G7MEAN_N.txt" )) espace_set_=true;
      }
    else if(espace_type=="Lateral_Tripod")
      {
	if(parseFiles(espace_dir_ + "G8VECS_N.txt",espace_dir_ + "G8MEAN_N.txt" )) espace_set_=true;
      }
    else
      ROS_ERROR("Invalid Eigenspace type given - no Eigenspace is set. Type has to be one of: Global, Tripod, Palmar_Pinch, Lateral, Writing_Tripod, Parallel_Extension, Adduction_Grip, Tip_Pinch or Lateral_Tripod. ");
  }
}//end namespace


