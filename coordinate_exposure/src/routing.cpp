
//Including the libraries, including the one for converting the coordinates between OpenCV and the ROS message type


#include <ros/ros.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <coordinate_exposure/coordinate_sets_exchange.h>
#include <coordinate_exposure/coordinate_conversion.h>


//Function calculates the stretch difference if it were swapped according to the passed iterators and returns it


double routeDifference(std::vector<std::vector<cv::Point2d> >::iterator front_iter, std::vector<std::vector<cv::Point2d> >::iterator back_iter)
{
  cv::Point2d first_pair_start, first_pair_end;
  cv::Point2d second_pair_start, second_pair_end;
  double distance_before, distance_after;

  
//Access the required coordinates using the passed iterators


  first_pair_start = front_iter->back();
  first_pair_end = (*(front_iter+1)).front();
  second_pair_start = back_iter->back();
  second_pair_end = (*(back_iter+1)).front();

 
//Calculation of the distances to be covered on the two routes


  distance_before = cv::norm(first_pair_start - first_pair_end) + cv::norm(second_pair_start - second_pair_end);
  distance_after = cv::norm(first_pair_start - second_pair_start) + cv::norm(first_pair_end - second_pair_end);

  
//Return the stretch difference of both routes


  return distance_after - distance_before;
}


//Function compares the distance savings by the various possible pairwise interchanges and performs the most promising as long as it reduces the distance at all; No commutation has been made, that is, no distance reductions are possible by paired exchange, the function return parameter assumes the logical value 0


bool iteration(std::vector<std::vector<cv::Point2d> >::iterator begin_iter, std::vector<std::vector<cv::Point2d> >::iterator end_iter)
{
  std::vector<std::vector<cv::Point2d> >::iterator front_iter = begin_iter, back_iter;
  std::vector<std::vector<cv::Point2d> >::iterator min_route_front_iter, min_route_back_iter;
  double delta_route;

  
//Maximum path saving to 0 preinitialize


  double min_delta_route = 0;

  
//Viewing of all route changes by paired route exchange


  do
  {
    for(back_iter = front_iter+1; back_iter != end_iter-1; back_iter++)
    {
      
//Determination of the length difference between previous route and alternative route


      delta_route = routeDifference(front_iter, back_iter);

      
//If the route of the last alternative route is greater than the previous one, this value and the iterators are saved



      if(delta_route < min_delta_route)
      {
        min_delta_route = delta_route;
        min_route_front_iter = front_iter;
        min_route_back_iter = back_iter;
      }
    }

    
//Incrementing the front pair (front section)


    front_iter++;
  }
  while(front_iter != end_iter-2);

  
//Carry out the paired exchange if at least one of the calculated changes entails a reduction in the distance


  if(min_delta_route < 0)
  {
    
//Sequence in which the intermediate sections are traversed, so that only the two sections included in the calculation are interchanged



    std::reverse(min_route_front_iter+1, min_route_back_iter+1);

    
//Also reverse the intermediate sections (ie, the start and end points are swapped)



    for(std::vector<std::vector<cv::Point2d> >::iterator reverse_iter = min_route_front_iter+1; reverse_iter != min_route_back_iter+1; reverse_iter++)
    {
      std::reverse(reverse_iter->begin(), reverse_iter->end());
    }

    
//Function indicates that a change has been made


    return true;
  }

  
//Function indicates that no change has been made due to the lack of potential savings


  else
  {
    return false;
  }
}


//Function which implements the method of pairing in the iterators



void pairwiseExchange(std::vector<std::vector<cv::Point2d> >::iterator begin_iter, std::vector<std::vector<cv::Point2d> >::iterator end_iter)
{
  bool terminator;

  
//Carry out the method of the pairwise reversal only if there are so many route sections that the connections of these can be interchanged



  if(begin_iter != end_iter && begin_iter+1 != end_iter && begin_iter+2 != end_iter && begin_iter+3 != end_iter)
  {
    
//Iteration without scheduling until no pairing allows more savings


    do
    {
      terminator = !iteration(begin_iter, end_iter);
    }
    while(terminator == false);
  }
}


//Callback function, which can always be activated when the service of the pairwise interchange is requested


bool rxCallback(coordinate_exposure::coordinate_sets_exchange::Request& req, coordinate_exposure::coordinate_sets_exchange::Response& res)
{
  coordinate_exposure::coordinate_sets req_coordinate_chain_set;
  coordinate_exposure::coordinate_sets res_coordinate_chain_set;
  std::vector<std::vector<cv::Point_<double> > > coordinate_chain_set;

 
//Vector with coordinate sets from Request


  req_coordinate_chain_set = req.data;

  
//Change the format of the coordinates


  decodeCoordinateSets(req_coordinate_chain_set, coordinate_chain_set);

 
//Optimize the way using the pairing


  pairwiseExchange(coordinate_chain_set.begin(), coordinate_chain_set.end());

  
//Reversing the format of the coordinates


  codeCoordinateSets(coordinate_chain_set, res_coordinate_chain_set);


//Write the result of the route planning in Response


  res.data = res_coordinate_chain_set;

 
//Logical 1 indicates that the service has been successfully performed


  return true;
}

int main(int argc, char** argv)
{
 
//Initialization of the node


  ros::init(argc, argv, "routing");

  
//Node objects with relative and private namespaces


  ros::NodeHandle node;

//Service with the provided callback function


  ros::ServiceServer server = node.advertiseService("routing", rxCallback);

  
//Waiting for callbacks to be triggered by received messages


  ROS_INFO("Offering the service 'routing'.");
  ros::spin();

  return 0;
}
