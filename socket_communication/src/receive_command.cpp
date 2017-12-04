#include <ros/ros.h>
#include"std_msgs/String.h"
#include"robot_communication/connection_msg.h"
#include"robot_communication/current_pos_robot.h"
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include<iostream>
#include <unistd.h>
#include<QString>
#include<QStringList>
#include<vector>
#define PORT  1001

using namespace std;
int m_socket;

char *convert(const std::string & s)
{
  char *pc = new char[s.size()+1];
  std::strcpy(pc, s.c_str());
  return pc;
}

//Function for making the socket communication with particular protocol
bool ClientConnect(const char* host, int port, int iIdSocket)

{
  sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(host);
  if(connect(iIdSocket, (sockaddr *) &addr, sizeof(sockaddr)) <0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool currentstatus(robot_communication::current_pos_robot & robot_pose_msg)
{
  //ros::Rate loop_rate(0.4);
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<robot_communication::current_pos_robot>("/robot_current_pos",1000); // creat publisher which publish a data on a topic "/robot_current_pos"
  

  char ReadString[5000];
  double x,y,z,c;
  bool got_it;
  QString toSplitString;

  got_it=recv(m_socket,ReadString,5000,0); // read the data from robot controller
  if(got_it==false)
  {
    return false;
  }
  else
  {
    toSplitString = QString::fromUtf8(ReadString);
    QStringList ReceiveList = toSplitString.split(" "); //convert data in QStringlist
    ReceiveList.removeFirst(); //remove "LESE" from the data
    for(int i = 0; i < ReceiveList.size(); i++)
    {
      if(ReceiveList.value(i).toDouble() == 0)
      {
        ReceiveList.removeAt(i); //Remove any empty spaces
        i = i - 1;
      }
    }

    //assign the values
    x = ReceiveList.value(0).toDouble();
    y = ReceiveList.value(1).toDouble();
    z = ReceiveList.value(2).toDouble();
    c = ReceiveList.value(3).toDouble();


    robot_pose_msg.PosX=x;
    robot_pose_msg.PosY=y;
    robot_pose_msg.PosZ=z;
    robot_pose_msg.PosC=c;
    pub.publish(robot_pose_msg); //publish the current position of the robot on a topic "/robot_current_pos"
    //loop_rate.sleep();
    return true;

  }


}



void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ros::Rate loop_rate(0.4);

  char c_Send[30];
  bool b_Send;

  robot_communication::current_pos_robot robot_pose_msg;
  vector<string> list;
  list.push_back(msg->data);

  std::vector<char*>  vc;

  std::transform(list.begin(), list.end(), std::back_inserter(vc), convert);

  for ( size_t i = 0 ; i < vc.size() ; i++ )
  {
    strcpy(c_Send,vc[i]);
  }
  for ( size_t i = 0 ; i < vc.size() ; i++ )
  {
    delete [] vc[i];
  }
  strcat(c_Send,"\r\n");

  b_Send = write(m_socket,c_Send,strlen(c_Send)); // send the data to the robot controller

  currentstatus(robot_pose_msg);


  list.erase (list.begin()+3);


}

int main(int argc, char **argv)
{
  // Set up ROS.

  ros::init(argc, argv, "receive_command"); //create a node name "receive_command"

  ros::NodeHandle n; //create a object for node handle
  // ros::Publisher status = n1.advertise<robot_communication::current_pos_robot>("/alive_positions",1000); // creat publisher which publish a data on a topic "/robot_current_pos"
  robot_communication::current_pos_robot robot_current_pos;

  int iPort = PORT;
  bool bConnected;
  const char* c_Host = "192.168.0.124"; //ip address of the robot controller

  m_socket = socket(AF_INET, SOCK_STREAM, 0); //create a socket
  bConnected = ClientConnect(c_Host,iPort,m_socket); //do connection with the socket

  if(bConnected == true)
  {
    ROS_INFO( "connected");
  }
  else
  {
    ROS_INFO("Notconnected");
  }

  //to check if its alive..."4,0,0,0,0,0,0,0,0\r\n"

  char check[30];
  bool send,rcv;
  strcpy(check,"4,0,0,0,0,0,0,0,0\r\n");
  ros::Rate loop_rate(0.4);


  do
  {
    ROS_INFO("checking if its alive...");
    send = write(m_socket,check,strlen(check));
    rcv=currentstatus(robot_current_pos);
    loop_rate.sleep();

  }
  while(rcv!=true);

  ROS_INFO("servo motor is on");

  ros::Subscriber sub = n.subscribe("/send_string", 10000, chatterCallback); // subscribe a topic "/send_string"

  ros::spin();
  close(m_socket); //close the socket

  return 0;

}



