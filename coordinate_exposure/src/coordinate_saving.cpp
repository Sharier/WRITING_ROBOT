
//Including the libraries, including the APIs for accessing files


#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <coordinate_exposure/coordinate_sets.h>
#include <opencv2/core/core.hpp>
#include <coordinate_exposure/coordinate_conversion.h>


//Default for the path of the file to write to


const std::string DEFAULT_FILE_NAME = "/home/ros/Bilder/robot_commands.txt";


//Global variable for the path of the file to be written to


std::string g_file_name;


//Always triggered callback function when a set of coordinate chains is received and stores these coordinates in a form suitable for the Toshiba TS2100 controller at the IAT


void rxCallback(const coordinate_exposure::coordinate_sets::ConstPtr& msg_coordinate_chain_set_ptr)
{
  std::vector<std::vector<cv::Point_<double> > > coordinate_chain_set;
  std::vector<std::vector<cv::Point2d> >::iterator c;
  std::vector<cv::Point_<double> > coordinate_chain;
  std::vector<cv::Point_<double> >::iterator p;
  std::ofstream file;
  double x, y;

  
//Update file name parameters


  g_file_name = ros::param::param("~file_name", DEFAULT_FILE_NAME);

  
//Conversion of the file string into C string


  const char* file_name = g_file_name.c_str();

  
//Open the file with write access rights


  file.open(file_name);

  
//Coordinate coordinates in the message datatype in OpenCV coordinates


  decodeCoordinateSets(*msg_coordinate_chain_set_ptr, coordinate_chain_set);

  
//First route


  c = coordinate_chain_set.begin();

  
//All coordinate chains processed


  while(c != coordinate_chain_set.end())
  {
    
//Set iterator to first coordinate


    if(c->begin() != c->end())
    {
      p = c->begin();
    }
    else
    {
      continue;
    }

   
//All the coordinates of the chain are formulated individually in strings for transfer to controllers; All strings are saved by separating them into separate lines


    while(true)
    {
     
//Abscissa and ordinate


      x = p->x;
      y = p->y;

     
//For the last coordinate of a coordinate chain, i.e., a link, the arm on the surface is guided to this coordinate and then raised to be able to drive the next coordinate chain in an elevated position


      if(p == c->end()-1)
      {
        file << "5," << x << "," << y << "," << "90,0,0,1,20,0" << std::endl;
        file << "1," << x << "," << y << "," << "100,0,0,1,20,0" << std::endl;
        break;
      }

      
//For the first coordinate of a coordinate chain, that is, a link, the arm is first moved over the surface to this (x, y) coordinate and then lowered


      else if(p == c->begin())
      {
        file << "1," << x << "," << y << "," << "100,0,0,1,20,0" << std::endl;
        file << "1," << x << "," << y << "," << "90,0,0,1,20,0" << std::endl;
      }

      
//The average coordinates can be normalized on the surface, whereby a rectilinear motion between the coordinates is prescribed


      else
      {
        file << "5," << x << "," << y << "," << "90,0,0,1,20,0" << std::endl;
      }

     
//Next coordinate


      p++;
    }

   
//Next coordinate chain


    c++;
  }

  
//Coordinate origin


  file << "1,-245,-592,150,0,0,1,20,0" << std::endl;

 
//Close the file again



  file.close();

  
//Information of the user


  ROS_INFO("Got a set of coordinate chain and saved the resulting strings in file %s.", file_name);
}

int main(int argc, char** argv)
{
  
//Initialization of the node


  ros::init(argc, argv, "coordinate_saving");

  
//Creation of a Node object


  ros::NodeHandle node;

  
//Subscriber with appropriate callback function


  ros::Subscriber sub = node.subscribe("coordinate/coordinate_chain_set", 1000, &rxCallback);

 
//Path to the file on the server parameter


  ros::param::param("~file_name", g_file_name, DEFAULT_FILE_NAME);

  
//Waiting for callbacks to be triggered by received messages


  ROS_INFO("Waiting for coordinates to save.");
  ros::spin();

  return 0;
}
