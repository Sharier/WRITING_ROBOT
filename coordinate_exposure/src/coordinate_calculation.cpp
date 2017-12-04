
//Including the libraries, including the message types for the transmission of coordinates individually or in a merged form


#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <iterator>
#include <boost/move/move.hpp>
#include <coordinate_exposure/coordinate.h>
#include <coordinate_exposure/coordinate_set.h>
#include <coordinate_exposure/coordinate_sets.h>
#include <coordinate_exposure/coordinate_sets_exchange.h>
#include <coordinate_exposure/coordinate_conversion.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

const int AHEAD = 0;
const int RIGHT = 1;

const double DINA4_LENGTH = 297.;
const double DINA4_WIDTH = 210.;


//Class which can extract relative positions from different types of messages, convert them to (world) coordinates and transmit them further


class CoordinateCalculator
{
private:
  ros::NodeHandle rel_node_;
  ros::NodeHandle pri_node_;
  ros::Subscriber coordinate_sets_sub_;
  ros::Subscriber coordinate_set_sub_;
  ros::Subscriber coordinate_sub_;
  ros::Publisher coordinate_sets_pub_;
  ros::Publisher coordinate_set_pub_;
  ros::Publisher coordinate_pub_;
  ros::ServiceClient routing_client_;
  cv::Point_<double> origin_diff_;

public:
  CoordinateCalculator()
  : pri_node_("~"), origin_diff_(-245, -592)
  {
    
//List of all topics with which the class is connected via publishes or subscribers


    coordinate_sets_sub_ = rel_node_.subscribe("relative_position/coordinate_chain_set", 1000, &CoordinateCalculator::positionSetsCallback, this);
    coordinate_set_sub_ = rel_node_.subscribe("relative_position/coordinate_chain", 1000, &CoordinateCalculator::positionSetCallback, this);
    coordinate_sub_ = rel_node_.subscribe("relative_position/coordinate", 1000, &CoordinateCalculator::positionCallback, this);
    coordinate_sets_pub_ = rel_node_.advertise<coordinate_exposure::coordinate_sets>("world_coordinate/coordinate_chain_set", 1000);
    coordinate_set_pub_ = rel_node_.advertise<coordinate_exposure::coordinate_set>("world_coordinate/coordinate_chain", 1000);
    coordinate_pub_ = rel_node_.advertise<coordinate_exposure::coordinate>("world_coordinate/coordinate", 1000);

  
//Client for the 'routing' service


    routing_client_ = rel_node_.serviceClient<coordinate_exposure::coordinate_sets_exchange>("routing");

    
//Creation of parameters on the parameter server to be accessed later, if not already present


    if(!pri_node_.hasParam("orientation"))
    {
      pri_node_.setParam("orientation", AHEAD);
    }
    if(!pri_node_.hasParam("length_x"))
    {
      pri_node_.setParam("length_x", DINA4_LENGTH);
    }
    if(!pri_node_.hasParam("length_y"))
    {
      pri_node_.setParam("length_y", DINA4_WIDTH);
    }
  }

 
//Calculation of the world coordinate from this relative position


  inline void coordinateCalculation(cv::Point_<double>& coordinate)
  {
    double x_scale, y_scale, tmp;
    int orientation;

    
//Update the parameters


    pri_node_.param<int>("orientation", orientation, AHEAD);
    pri_node_.param<double>("length_x", x_scale, DINA4_LENGTH);
    pri_node_.param<double>("length_y", y_scale, DINA4_WIDTH);

  
//Make sure it is a relative position, because absolute positions could be unintentionally converted into coordinates outside the robot's safety range


    if(coordinate.x > 1  || coordinate.y > 1)
    {
      ROS_ERROR("Received an absolute position. Can not handle those ones.");
    }
    else
    {
     
//Depending on the orientation conversion between the coordinate systems up to the world coordinate


      if(orientation == AHEAD || orientation < 0)
      {
        coordinate.x = coordinate.x * x_scale;
        coordinate.y = (1 - coordinate.y) * y_scale;
        coordinate = coordinate + origin_diff_;
      }
      else
      {
        tmp = coordinate.x;
        coordinate.x = coordinate.y * x_scale;
        coordinate.y = tmp * y_scale;
        coordinate = coordinate + origin_diff_;
      }
    }
  }

 
//Callback function for translating a set of sets of relative positions to (world) coordinates


  void positionSetsCallback(const coordinate_exposure::coordinate_sets::ConstPtr& msg_coordinate_chain_set_ptr)
  {
    std::vector<std::vector<cv::Point_<double> > > coordinate_chain_set;
    std::vector<std::vector<cv::Point2d> >::iterator c;
    std::vector<cv::Point_<double> >::iterator p;
    coordinate_exposure::coordinate_sets msg_coordinate_chain_set;
    coordinate_exposure::coordinate_sets_exchange srv_routing;

  
//Set of coordinate chains in the ROS message form is converted into vector with vectors from OpenCV coordinates


    decodeCoordinateSets(*msg_coordinate_chain_set_ptr, coordinate_chain_set);

    
//In each item set, convert each item individually to (world) coordinate


    for(c = coordinate_chain_set.begin(); c != coordinate_chain_set.end(); c++)
    {
      for(p = c->begin(); p!= c->end(); p++)
      {
        coordinateCalculation(*p);
      }
    }

   
//Conversion into the ROS message data type; Request by placing coordinates in the object to be transferred


    codeCoordinateSets(coordinate_chain_set, srv_routing.request.data);

    
//If service 'routing' is offered, use this to optimize the order; Output whether this was done


    if(routing_client_.call(srv_routing))
    {
      coordinate_chain_set.clear();
      decodeCoordinateSets(srv_routing.response.data, coordinate_chain_set);
    }
    else
    {
      ROS_INFO("Service 'routing' is not available. Publishing the coordinate chains in suboptimal order.");
    }

    
//Check all coordinate chains to determine whether the last coordinate matches the initial coordinate of the sequence chain to reduce the chains to one


    c = coordinate_chain_set.begin();
    while(c != coordinate_chain_set.end()-1)
    {
      if(*(c->end()-1) == *((c+1)->begin()))
      {
        c->reserve(c->size() + (c+1)->size() - 1);
        c->insert(c->end(), boost::make_move_iterator((c+1)->begin()+1), boost::make_move_iterator((c+1)->end()));
        if(c+1 == coordinate_chain_set.end()-1)
        {
          coordinate_chain_set.pop_back();
        }
        else
        {
          coordinate_chain_set.erase(c+1);
        }
      }
      else
      {
        c++;
      }
    }

    
//Filling the message object, which is used to transfer the coordinate chains


    codeCoordinateSets(coordinate_chain_set, msg_coordinate_chain_set);

   
//Publish the set of target coordinate chains


    ROS_INFO("Transferred the received set of relative position chains into a set of coordinate chains and published the results on output topic.");
    coordinate_sets_pub_.publish(msg_coordinate_chain_set);
  }

  
//Callback function to translate from a set of relative positions to (world) coordinates


  void positionSetCallback(const coordinate_exposure::coordinate_set::ConstPtr& msg_coordinate_chain_ptr)
  {
    std::vector<cv::Point_<double> > coordinate_chain;
    std::vector<cv::Point_<double> >::iterator p;
    coordinate_exposure::coordinate_set msg_coordinate_chain;

    
//The amount of coordinates is transferred from the present message into a vector with OpenCV points


    decodeCoordinateSet(*msg_coordinate_chain_ptr, coordinate_chain);

    
//Convert each position individually to (world) coordinates


    for(p = coordinate_chain.begin(); p != coordinate_chain.end(); p++)
    {
      coordinateCalculation(*p);
    }

 
//Reconciliation to the ROS message data type


    codeCoordinateSet(coordinate_chain, msg_coordinate_chain);

   
//Publish the set of target coordinates


    ROS_INFO("Transferred the received set of relative positions into a set of coordinates and published the results on output topic.");
    coordinate_set_pub_.publish(msg_coordinate_chain);
  }

  
//Callback function for transferring a relative position to a (world) coordinate


  void positionCallback(const coordinate_exposure::coordinate::ConstPtr& msg_coordinate_ptr)
  {
    cv::Point2d coordinate;
    coordinate_exposure::coordinate msg_coordinate;

   
//Coordinate from the message is packaged in OpenCV-Coordinate


    decodeCoordinate(*msg_coordinate_ptr, coordinate);

   
//Convert relative position to (world) coordinate


    coordinateCalculation(coordinate);

    
//Reconciliation to the ROS message data type


    codeCoordinate(coordinate, msg_coordinate);

  
//Publish the target coordinate


    coordinate_pub_.publish(msg_coordinate);
    ROS_INFO("Transferred all received relative positions into coordinates and published the results on output topic.");
  }
};

int main(int argc, char** argv)
{
  
//Initialization of the node


  ros::init(argc, argv, "coordinate_calculation");

  
//Calling an object of the class responsible for the coordinate calculation


  CoordinateCalculator cc;

  
//Waiting for callbacks to be triggered by received messages


  ROS_INFO("Waiting for relative positions to convert into coordinates.");
  ros::spin();

  return 0;
}
