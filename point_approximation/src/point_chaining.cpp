//Including the libraries, including OpenCV and message types used by the node
#include <ros/ros.h>
#include <stdlib.h>
#include <vector>
#include <set>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <coordinate_exposure/coordinate_sets.h>
#include <coordinate_exposure/coordinate_conversion.h>

static const std::string OPENCV_WINDOW = "Approximated image";

const bool REMOVE = true;
const bool KEEP = false;

const int PIXEL_AMOUNT_DEFAULT = 10;

//Class that provides the ordinal relation for the OpenCV points for storage in set
class PointOrder
{
public:
  bool operator() (cv::Point pt_a, cv::Point pt_b)
  {
    if(pt_a.x > pt_b.x)
    {
      return 0;
    }
    else if(pt_a.x == pt_b.x)
    {
      if(pt_a.y >= pt_b.y)
      {
        return 0;
      }
      else
      {
        return 1;
      }
    }
    else
    {
      return 1;
    }
  }
};

//Function that tests to a passed 3x3 matrix, whether the pixel is a node or intersection point in the middle
bool isIntersectionPoint(const cv::Mat_<unsigned char>& kernel)
{
  int first_term, second_term;
  //The initial requirement is that the pixel is occupied by a 1
  if(kernel(1,1) == 1)
  {
    //Consideration of the values for both condition variables
    first_term = (kernel(0, 0) == 0 && kernel(0, 1) == 1) + (kernel(0, 1) == 0 && kernel(0, 2) == 1) +
                 (kernel(0, 2) == 0 && kernel(1, 2) == 1) + (kernel(1, 2) == 0 && kernel(2, 2) == 1) +
                 (kernel(2, 2) == 0 && kernel(2, 1) == 1) + (kernel(2, 1) == 0 && kernel(2, 0) == 1) +
                 (kernel(2, 0) == 0 && kernel(1, 0) == 1) + (kernel(1, 0) == 0 && kernel(0, 0) == 1);
    second_term = kernel(0, 0) * kernel(0, 1) * kernel(0, 2) + kernel(0, 1) * kernel(0, 2) * kernel(1, 2) +
                  kernel(0, 2) * kernel(1, 2) * kernel(2, 2) + kernel(1, 2) * kernel(2, 2) * kernel(2, 1) +
                  kernel(2, 2) * kernel(2, 1) * kernel(2, 0) + kernel(2, 1) * kernel(2, 0) * kernel(1, 0) +
                  kernel(2, 0) * kernel(1, 0) * kernel(0, 0) + kernel(1, 0) * kernel(0, 0) * kernel(0, 1);
   // int third_term= (kernel(0,1)*kernel(1,2)*kernel(2,1));
   // int fourth_term= (kernel(1,0)*kernel(1,2)*kernel(2,1));

    //Verify that the calculated values meet the conditions and return the corresponding truth value
    if(first_term >= 3 && second_term == 0)
    {
      return true;
    }
  }

 //This is not a node (-> return of false) because not all of the above criteria are met

  return false;
}

//Function, which tests to a passed 3x3 matrix, whether the image point in the center is an end point
bool isEndPoint(const cv::Mat_<unsigned char>& kernel)
{
  int first_term, second_term;
;

  //The initial requirement is that the pixel considered has a value of 1
  if(kernel(1,1) == 1)
  {
    //Calculation of the values with regard to the criteria to be considered
    first_term = cv::sum(kernel)[0];
    second_term = (kernel(0, 0) == 0 && kernel(0, 1) == 1) + (kernel(0, 1) == 0 && kernel(0, 2) == 1) +
                  (kernel(0, 2) == 0 && kernel(1, 2) == 1) + (kernel(1, 2) == 0 && kernel(2, 2) == 1) +
                  (kernel(2, 2) == 0 && kernel(2, 1) == 1) + (kernel(2, 1) == 0 && kernel(2, 0) == 1) +
                  (kernel(2, 0) == 0 && kernel(1, 0) == 1) + (kernel(1, 0) == 0 && kernel(0, 0) == 1);

    //int third_term= (kernel(0,1)*kernel(1,2)*kernel(2,1));
    //int fourth_term= (kernel(1,0)*kernel(1,2)*kernel(2,1));

    //Comparison of the values with the specifications and return of logical 1 on match
    if(first_term >= 2 && first_term <= 3&& second_term == 1 )   //Overwriting but improved at first_term>=3&& second_term<=7
    {
      return true;
    }
  }

  //This is not an endpoint (-> return of false) because not all of the above criteria are met
  return false;
}

//Class for determining the relative positions, by means of which the lettering of the image can be linearly approximated
class CoordinateDetector
{
private:
  ros::NodeHandle rel_node_;
  ros::NodeHandle pri_node_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::Publisher ros_pub_;
  cv::Mat machined_;
  std::set<cv::Point, PointOrder> pts_memory_;

  //Function, which determines the response to the receipt of an image message
  void rxCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    std::vector<std::vector<cv::Point_<double> > > pos_chain_set;
    coordinate_exposure::coordinate_sets msg_pos_chain_set;
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat src;

    //Error-protected attempt to transfer the image content of the message to OpenCV-compatible version

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
      return;
    }

    //Save source image to Mat data type
    src = cv_ptr->image;

    //Addition of a boundary layer to pixels in order to examine the outer points in their own way
    cv::copyMakeBorder(src, this->machined_, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar_<unsigned char>(0));

    //Determination of relative positions with which the image can be reconstructed roughly afterwards
    positionApproximation(pos_chain_set);

    //Approximation image by connecting points linearly; Display results and save them to png file
    cv::Mat drawing = cv::Mat(src.size(), CV_8UC1, cv::Scalar(255));
    for(int a = 0; a < pos_chain_set.size(); a++)
    {
      std::vector<cv::Point_<double> > d = pos_chain_set[a];
      for(int b = 0; b < d.size()-1; b++)
      {
        cv::line(drawing, cv::Point(src.cols*d[b].x, src.rows*d[b].y), cv::Point(src.cols*d[b+1].x, src.rows*d[b+1].y), cv::Scalar(0), 2, 8, 0);
      }
    }
    cv::imwrite("/home/ros/Bilder/approximated_image.png", drawing);
    cv::imshow("approximated image", drawing);
    cv::waitKey(0);

     //Information of the user about the results of the analysis
    if(pos_chain_set.empty())
    {
      ROS_ERROR("Found no coordinates to publish.");
    }
    else
    {
      ROS_INFO("Approximated the writings successfully with a set of relative position chains. Publishing the result.");
    }

  //Convert to portable format and publish the results
    codeCoordinateSets(pos_chain_set, msg_pos_chain_set);
    ros_pub_.publish(msg_pos_chain_set);
  }

public:
  int pixel_amount_;

  CoordinateDetector()
   //Use ImageTransport object it_ to exchange messages with image content

    : it_(rel_node_), pri_node_("~")
  {
    //Determining the subscribed topic as well as the action steps subsequent to receiving a message by specifying the callback function; Create a publisher for the topic on which the results are to be published

    img_sub_ = it_.subscribe("image/writings", 1, &CoordinateDetector::rxCallback, this);
    ros_pub_ = rel_node_.advertise<coordinate_exposure::coordinate_sets>("relative_position/coordinate_chain_set", 1000);

    //Set the parameters of the length of the approximation in pixels to the parameter server
    if(!pri_node_.hasParam("approximation/pixel_amount"))
    {
      pri_node_.setParam("approximation/pixel_amount", PIXEL_AMOUNT_DEFAULT);
    }
  }

  //Function receives a starting point from which it follows the neighboring pixels until an end or node point is reached; The path taken is stored, whereby approximation results from the fact that the point is stored only at a certain distance from the predecessor point

  std::vector<cv::Point_<double> > positionChaining(const cv::Point& start_pt, bool remove_start_pt = REMOVE)
  {
    std::vector<cv::Point_<double> > pos_chain;
    cv::Point_<double> current_pos;
    double x, y;

   //Start pixel counter at 0
    int pixel_counter = 0;

    //Definition of a variable to remember the predecessor pixel
    cv::Point predecessor = cv::Point(0, 0);

   //Set the initialization matrix to 0
    cv::Mat invert = cv::Mat(this->machined_.size(), this->machined_.type(), cv::Scalar(0));

   //Determine the number of rows and columns, which are 2 lower than those of the edited image, since edge was added to the editing image

    int rows = this->machined_.rows - 2;
    int cols = this->machined_.cols - 2;

   //If the source image has only one pixel, the execution of the function is free

    if(rows == 1 || cols == 1)
    {
      ROS_ERROR("Image processing is not possible since image has just one pixel.");
      return pos_chain;
    }

    //Relative start position in the vector
    x = static_cast<double>(start_pt.x) / (cols+1);
    y = static_cast<double>(start_pt.y) / (rows+1);
    current_pos = cv::Point_<double>(x, y);
    pos_chain.push_back(current_pos);

    //For easier access to the pixels, a Mat_ object is used
    cv::Mat_<unsigned char> kernel = cv::Mat_<unsigned char>(3,3);

   //The current point and the reference point are initialized with the starting point
    cv::Point reference_pt = start_pt;
    cv::Point current_pt = reference_pt;

    //3x3-matrix of the start pixel with the neighboring pixels from the image
    kernel = this->machined_(cv::Range(start_pt.y-1, start_pt.y+2), cv::Range(start_pt.x-1, start_pt.x+2));

   //If the function is set to 0 that the pixel of the start point of the concatenation is to be set to 0, this notation is made preventively

    if(remove_start_pt == REMOVE)
    {
      invert.at<unsigned char>(start_pt.y, start_pt.x) = 1;
    }

 //Follow the path

    while(true)
    {
     //Checking all adjacent image points to the value of 1 to find the nearest point of the chain, the horizontal / vertical ones being viewed in front of the diagonal adjacent pixels; Change of ordinate and abscissa according to the following point

      if(kernel(1, 0) != 0 && predecessor != cv::Point(-1, 0))
      {
        current_pt.x--;
        predecessor = cv::Point(1, 0);
      }
      else if(kernel(0, 1) != 0 && predecessor != cv::Point(0, -1))
      {
        current_pt.y--;
        predecessor = cv::Point(0, 1);
      }
      else if(kernel(1, 2) != 0 && predecessor != cv::Point(1, 0))
      {
        current_pt.x++;
        predecessor = cv::Point(-1, 0);
      }
      else if(kernel(2, 1) != 0 && predecessor != cv::Point(0, 1))
      {
        current_pt.y++;
        predecessor = cv::Point(0, -1);
      }
      else if(kernel(0, 0) != 0 && predecessor != cv::Point(-1, -1) && predecessor != cv::Point(-1, 0) && predecessor != cv::Point(0, -1))
      {
        current_pt.x--;
        current_pt.y--;
        predecessor = cv::Point(1, 1);
      }
      else if(kernel(0, 2) != 0 && predecessor != cv::Point(1, -1) && predecessor != cv::Point(1, 0) && predecessor != cv::Point(0, -1))
      {
        current_pt.x++;
        current_pt.y--;
        predecessor = cv::Point(-1, 1);
      }
      else if(kernel(2, 2) != 0 && predecessor != cv::Point(1, 1) && predecessor != cv::Point(1, 0) && predecessor != cv::Point(0, 1))
      {
        current_pt.x++;
        current_pt.y++;
        predecessor = cv::Point(-1, -1);
      }
      else if(kernel(2, 0) != 0 && predecessor != cv::Point(-1, 1) && predecessor != cv::Point(-1, 0) && predecessor != cv::Point(0, 1))
      {
        current_pt.x--;
        current_pt.y++;
        predecessor = cv::Point(1, -1);
      }
      else
      {
        machined_.at<unsigned char>(current_pt.y, current_pt.x) = 0;
        return pos_chain;
      }

       // Counter for the number of pixels approximated by the current line
      pixel_counter++;

     //Matrix with current element and direct neighbor elements
      kernel = this->machined_(cv::Range(current_pt.y-1, current_pt.y+2), cv::Range(current_pt.x-1, current_pt.x+2));

      //If an endpoint is hit, store this point in the vector and then stop the chaining; Pixels in inverting matrix to 1

      if(isEndPoint(kernel))
      {
        invert.at<unsigned char>(current_pt.y, current_pt.x) = 1;
        current_pos.x = static_cast<double>(current_pt.x) / (cols+1);
        current_pos.y = static_cast<double>(current_pt.y) / (rows+1);
        pos_chain.push_back(current_pos);
        this->machined_ = this->machined_ - invert;
        return pos_chain;
      }

     //If you hit a node, store this point in vector and abort chaining; Keep pixels in their original state

      else if(isIntersectionPoint(kernel))
      {
        current_pos.x = static_cast<double>(current_pt.x) / (cols+1);
        current_pos.y = static_cast<double>(current_pt.y) / (rows+1);
        pos_chain.push_back(current_pos);
        this->machined_ = this->machined_ - invert;
        pts_memory_.insert(current_pt);
        return pos_chain;
      }

      //If you hit the former node, this point must also be stored in the current curve in order to maintain the required appearance; Keep pixels in their original state
      else if(pts_memory_.find(current_pt) != pts_memory_.end())
      {
        current_pos.x = static_cast<double>(current_pt.x) / (cols+1);
        current_pos.y = static_cast<double>(current_pt.y) / (rows+1);
        pos_chain.push_back(current_pos);
        this->machined_ = this->machined_ - invert;
        return pos_chain;
      }

     //If the point is reached, the point is also stored and the chaining is interrupted; Pixel does not need to be edited in the inversion matrix because this has already been done with the remove_start_pt option
      else if(current_pt == start_pt)
      {
        current_pos.x = static_cast<double>(current_pt.x) / (cols+1);
        current_pos.y = static_cast<double>(current_pt.y) / (rows+1);
        pos_chain.push_back(current_pos);
        this->machined_ = this->machined_ - invert;
        return pos_chain;
      }

     //If the distance from the point to the last stored point exceeds the specified approximation range, the point is stored in the vector; Pixels in inverting matrix to 1
      else if(pixel_counter >= pixel_amount_)
      {
        invert.at<unsigned char>(current_pt.y, current_pt.x) = 1;
        current_pos.x = static_cast<double>(current_pt.x) / (cols+1);
        current_pos.y = static_cast<double>(current_pt.y) / (rows+1);
        pos_chain.push_back(current_pos);
        reference_pt = current_pt;
        pixel_counter = 0;
      }

      //For the remaining points, the pixel in the inverting matrix is set to 1 and continues as usual with the concatenation
      else
      {
        invert.at<unsigned char>(current_pt.y, current_pt.x) = 1;
      }
    }
  }

  //Function for determining relative positions with which a skeletal image can be reconstructed by linear interpolation

  void positionApproximation(std::vector<std::vector<cv::Point_<double> > >& pos_chain_set)
  {
    std::vector<cv::Point_<double> > pos_chain;
    cv::Mat_<unsigned char> kernel;
    int i, j;

    //Use the value on the parameter server for the distance of the approximation points

    pri_node_.param<int>("approximation/pixel_distance", pixel_amount_, PIXEL_AMOUNT_DEFAULT);

    //Run all lines to the outer edges
    for(i = 1; i < this->machined_.rows-1; i++)
    {
      //Start each line with the second column
      j = 1;

      //Run until the last column
      do
      {
        //Matrix with current element and direct neighbor elements
        kernel = this->machined_(cv::Range(i-1, i+2), cv::Range(j-1, j+2));

        //If it is an endpoint, use point chaining, increment the column counter, and set the pixel value of the endpoint to 0 after concatenation so that it is not considered again

        if(isEndPoint(kernel))
        {
          pos_chain_set.push_back(positionChaining(cv::Point(j, i), REMOVE));
          j++;
        }

        //If it is a node, perform point concatenation, but remain at the same point as it continues in several directions; Point for later purposes as an earlier node
        else if(isIntersectionPoint(kernel))
        {
          pos_chain_set.push_back(positionChaining(cv::Point(j, i), KEEP));
          pts_memory_.insert(cv::Point(j, i));
        }

        //If it is a former node, also perform point chaining and remain at the point
        else if(pts_memory_.find(cv::Point(j, i)) != pts_memory_.end())
        {
	  pos_chain_set.push_back(positionChaining(cv::Point(j, i), KEEP));
	  kernel = this->machined_(cv::Range(i-1, i+2), cv::Range(j-1, j+2));
	  if(isEndPoint(kernel))
	  {
	    pos_chain_set.push_back(positionChaining(cv::Point(j, i), REMOVE));
	  }
	  else
	  {
	    machined_.at<unsigned char>(i, j) = 0;
	  }
	  j++;
        }

       //If it is another point, go to the next pixel
        else
        {
          j++;
        }
      }
      while(j < this->machined_.cols-1);
    }

   
//A further double loop, which is supposed to turn over remaining writings in relative positions


    for(i = 1; i < this->machined_.rows-1; i++)
    {
      j = 1;

      do
      {
       
//If the pixel is occupied by a 0, it can be skipped / omitted


        if(this->machined_.at<unsigned char>(i, j) == 0)
        {
          j++;
          continue;
        }

        
//Still use a 1 occupied pixel for starting a chaining and then proceed to the next pixel


        else
        {
          pos_chain = positionChaining(cv::Point(j, i), REMOVE);

          
//Only store the position chain if it is really a chain of several positions instead of a single position



          if(pos_chain.size() >= 2)
          {
            pos_chain_set.push_back(pos_chain);
          }

         
//Next column

          j++;
        }
      }
      while(j < this->machined_.cols-1);
    }
  }
};

int main(int argc, char** argv)
{
  
//Initialization of the node


  ros::init(argc, argv, "point_chaining");

  
//Creation of an object of the class created for the execution of the code


  CoordinateDetector cd;

  
//Waiting for callbacks to be triggered by received messages


  ROS_INFO("Waiting for published binary images with writings.");
  ros::spin();

  return 0;
}
