
//Including libraries, including libraries for transferring images and OpenCV points


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <paper_detection/point_set.h>
#include <paper_detection/point_conversion.h>


//Default settings for upper and lower limit of edge detection


const double DEFAULT_LOWER_THRESH = 13;
const double DEFAULT_UPPER_THRESH = 30;


//Function determines the cosine of the angle at the point pt_a, which is spanned to the points pt_b and pt_c


double cosAngle(cv::Point pt_a, cv::Point pt_b, cv::Point pt_c)
{
    double distance_ab, distance_ac, distance_bc, cos_alpha;

    
//Determine the distances between the points using the Euclidean metric


    distance_ab = cv::norm(cv::Mat(pt_a), cv::Mat(pt_b));
    distance_ac = cv::norm(cv::Mat(pt_a), cv::Mat(pt_c));
    distance_bc = cv::norm(cv::Mat(pt_b), cv::Mat(pt_c));

    
//Application of the cosine theorem for the determination of the cosine of the angle


    cos_alpha = (distance_ac*distance_ac + distance_ab*distance_ab - distance_bc*distance_bc) / (2 * distance_ac * distance_ab + 1e-10);

    
//Return the cosine from the angle spanned from pt_a to pt_b and pt_c


    return cos_alpha;
}


//Function which compares the surfaces of two elements given by points. If the area of the first shape is larger than that of the second form, true is returned and vice versa


inline bool isAreaGreater(std::vector<cv::Point> first_rect, std::vector<cv::Point> second_rect)
{
  return cv::contourArea(first_rect) > cv::contourArea(second_rect);
}


//Class for subscribing an image topic, then finding all the rectangles in it, of which the area largest rectangle is published on another topic


class RectangleDetector
{
private:
  ros::NodeHandle rel_node_;
  ros::NodeHandle pri_node_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::Publisher pts_pub_;
  cv::Mat src_;
  std::vector<std::vector<cv::Point> > rects_;

public:
  RectangleDetector()
    : it_(rel_node_), pri_node_("~")
  {
   
//Definition of the topics from / on which subscribed or advertised as well as the callback functions to be called


    img_sub_ = it_.subscribe("image/environment", 1, &RectangleDetector::rxCallback, this);
    pts_pub_ = rel_node_.advertise<paper_detection::point_set>("rectangle/element", 1000);

    
//Generation of the parameters, if not already present, using the NodeHandle for the private namespace


    if(!pri_node_.hasParam("lower_threshold"))
    {
      pri_node_.setParam("lower_threshold", DEFAULT_LOWER_THRESH);
    }
    if(!pri_node_.hasParam("upper_threshold"))
    {
      pri_node_.setParam("upper_threshold", DEFAULT_UPPER_THRESH);
    }
  }

  
//Deconstructor, used to delete the (private) parameters


  ~RectangleDetector()
  {
    if(!pri_node_.deleteParam("lower_threshold"))
    {
      ROS_WARN("Parameter for lower threshold has been deleted.");
    }
    if(!pri_node_.deleteParam("upper_threshold"))
    {
      ROS_WARN("Parameter for upper threshold has been deleted.");
    }
  }

  
//Callback function that manages the response to receiving images over the subscribed topic


  void rxCallback(const sensor_msgs::ImageConstPtr& ros_img_ptr)
  {
    cv_bridge::CvImagePtr cv_img_ptr;
    paper_detection::point_set elem_msg;

    
//Transfer of received ROS internal image to OpenCV-compatible image of format BGR8, if output fails

    
//An ROS error message


    try
    {
      cv_img_ptr = cv_bridge::toCvCopy(ros_img_ptr, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
      return;
    }

    
//Save the OpenCV compatible source image as a member


    src_ = cv_img_ptr->image;

   
//Find rectangles in the source image


    findRectangles();

    
//If the rectangle is found, follow the steps described above, otherwise output a warning message


    if(!rects_.empty())
    {
      
//Inform the user that rectangles have been found


      ROS_INFO("Rectangles found. The greatest one will be published.");

      
//Descending sorting of the rectangle by area content


      sortByArea();

      
//Draw, display and save the largest rectangle with red color in the input image


      cv::Mat drawing = src_;
      cv::line(drawing, rects_.front()[0], rects_.front()[1], cv::Scalar(0, 0, 255), 2, 8, 0);
      cv::line(drawing, rects_.front()[1], rects_.front()[2], cv::Scalar(0, 0, 255), 2, 8, 0);
      cv::line(drawing, rects_.front()[2], rects_.front()[3], cv::Scalar(0, 0, 255), 2, 8, 0);
      cv::line(drawing, rects_.front()[3], rects_.front()[0], cv::Scalar(0, 0, 255), 2, 8, 0);
      cv::imwrite("/home/ros/Bilder/environment.png", drawing);
      cv::imshow("rectangle", drawing);
      cv::waitKey(0);


      
//Transfer of the vector with OpenCV points into appropriate structure for transmission


      codePointSet(rects_.front(), elem_msg);

      
//Publish the points of the rectangle with current time stamp


      elem_msg.header.stamp = ros::Time::now();
      pts_pub_.publish(elem_msg);
    }
    else
    {
      ROS_WARN("No rectangles found.");
    }
  }

  
//Method for detecting any rectangles occurring in the source image


  void findRectangles()
  {
    cv::Mat src_gray, blurred;
    cv::Mat detected_edges;
    cv::Mat kernel;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> approx;
    double lower_thresh, upper_thresh;

    
//Source image in grayscale


    cv::cvtColor(src_, src_gray, CV_BGR2GRAY);

    
//Opacity of the image using the median to improve the subsequent edge detection by a more ordered contrast


    cv::medianBlur(src_gray, blurred, 9);

    
//Upper and lower limits for subsequent edge detection


    if(!pri_node_.getParam("lower_threshold", lower_thresh) || !pri_node_.getParam("upper_threshold", upper_thresh))
    {
      ROS_ERROR("Looking for unavailable threshold parameters.");
    }

    
//Edge detection with the Canny algorithm


    cv::Canny(blurred, detected_edges, lower_thresh, upper_thresh, 3);

    
//Conduction of dilatation can close smaller gaps between the determined edge lines without significantly displacing them


    kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(detected_edges, detected_edges, kernel, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT);

    
//Development of complete outlines from loose edges


    cv::findContours(detected_edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    
//Remove the rectangle from the last pass


    rects_.clear();

    
//Loop over all outlines to select the rectangle


    for(int i = 0; i < contours.size(); i++)
    {
      
//Approximation of the contours by fewer points by giving a distance of the approximation to the output contour which results linearly from constant factor 1 plus 0.8% of the length of the contour


      cv::approxPolyDP(contours[i], approx, 1 + cv::arcLength(contours[i], true)*0.008, true);

      
//Continue only if the approximated element is represented by four vertices that span a convex set with a sum of the page lengths of over 100 pixels (so that not a few very small rectangles load the additional runtime)


      if(approx.size() == 4 && cv::isContourConvex(approx) && cv::arcLength(approx, true) > 100.)
      {
        double cosine_angle;

        double max_cosine_angle = 0.;

       
//Calculate the cos from the angle of each corner of the element and store the largest of these angles


        for(int j = 1; j < 5; j++)
        {
          cosine_angle = std::abs(cosAngle(approx[j%4], approx[j-1], approx[(j+1)%4]));
          max_cosine_angle >= cosine_angle ? : max_cosine_angle = cosine_angle;
        }

        
//Selection of elements by angle by storing only those shapes in the final variable whose maximum cos of an angle does not exceed 0.1219 (in this way the angular range 83-97 ° is allowed)


        if(max_cosine_angle < 0.1219)
        {
          rects_.push_back(approx);
        }
      }
    }
  }

 
//Function for descending sorting according to area content


  void sortByArea()
  {
    std::sort(rects_.begin(), rects_.end(), isAreaGreater);
  }
};

int main(int argc, char** argv)
{
 
//Creation of the node


  ros::init(argc, argv, "rectangle_detector");

  
//Creation of an object of the class created for the execution of the code


  RectangleDetector rd;

 
//Waiting for callbacks through received messages


  ROS_INFO("Waiting for published images on the subscribed topic.");
  ros::spin();

  return 0;
}
