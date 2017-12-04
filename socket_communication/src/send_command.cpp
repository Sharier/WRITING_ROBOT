#include <ros/ros.h>
#include"std_msgs/String.h"
#include<vector>
#include<string.h>
#include<fstream>
#include<sstream>
#include<robot_communication/connection_msg.h>


int main(int argc, char **argv)
{


  std::string line;

  std::ifstream myfile;


  ros::init(argc, argv, "send_command"); //create a node name "send_command"
  ros::NodeHandle n;
  ros::Publisher send_command_pub = n.advertise<std_msgs::String>("/send_string", 10000);// create publisher which publish a data on a topic "/send_string"


  ros::Rate loop_rate(0.2);


  std_msgs::String msg;

  char c_send[3000];


  while (ros::ok())
  {

    myfile.open("/home/ros/bilder/robot_command.txt");

    if(!myfile) //Always test the file open.
    {
      ROS_INFO("Error opening output file");
      system("pause");
      return -1;
    }

    while(std::getline(myfile,line))
    {

      strcpy(c_send, line.c_str());
      msg.data = c_send;

      send_command_pub.publish(msg); //publish the data on a topic "/send_string"

      // ros::spinOnce();
      loop_rate.sleep();


    }

  }
  return 0;

}
