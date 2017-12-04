
//Including the libraries, including units around the time synchronization of messages and the libraries for the messages


#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <paper_detection/point_set.h>
#include <paper_detection/point_conversion.h>

static const std::string OPENCV_WINDOW = "Centered image";

const ros::Duration MAX_ELEM_DELAY = ros::Duration(10.);


//A built-up class, which subscribes the image and points together, the transformation matrix is determined and applied from the points so that the area of the image bounded by the points is extended over the target image


class Centering
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  message_filters::Subscriber<sensor_msgs::Image> img_sub_;
  message_filters::Subscriber<paper_detection::point_set> pts_sub_;
  image_transport::Publisher img_pub_;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, paper_detection::point_set> > sync_;
  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, paper_detection::point_set> timer_;

public:
  Centering()
    
//Definition of subscribed topics; Creation of a time synchronizer unit


    : it_(nh_), pts_sub_(nh_, "element", 1), img_sub_(nh_, "image/environment", 1), timer_(1),
      sync_(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, paper_detection::point_set>(2), img_sub_, pts_sub_)
  {
    timer_.setMaxIntervalDuration(MAX_ELEM_DELAY);

    
//Combination of the messages subscribed to the two different topics and transfer to the noted callback function


    sync_.registerCallback(boost::bind(&Centering::rxCallback, this, _1, _2));

  
//Notation of the topic on which publisht is


    img_pub_ = it_.advertise("image/centered", 1);
  }

 
//Callback function, which is always called when both dots and images are received and the subsequent process is controlled


  void rxCallback(const sensor_msgs::ImageConstPtr& ros_src_img_ptr, const paper_detection::point_set::ConstPtr& elem_msg_shared_ptr)
  {
    cv_bridge::CvImagePtr cv_src_img_ptr;
    cv_bridge::CvImage cv_dst_img;
    sensor_msgs::ImagePtr ros_dst_img_ptr;
    cv::Mat src, dst;
    cv::Mat transform_matrix;
    std::vector<cv::Point> elem;
    int top_right, bottom_left;

    int bottom_right = 0;
    int max_y = 0;

   
//Transfer of received ROS internal image to OpenCV-compatible image of format BGR8, if output fails


   
//An ROS error message


    try
    {
      cv_src_img_ptr = cv_bridge::toCvCopy(ros_src_img_ptr, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Problem with cv_bridge regarding the conversion from CV to ROS format: %s", e.what());
      return;
    }

    
//Save source image


    src = cv_src_img_ptr->image;

    
//Receive vector transformed at points in vector at OpenCV points


    decodePointSet(*elem_msg_shared_ptr, elem);

   
//If the received message is empty, print the error message


    if(elem.empty())
    {
      ROS_ERROR("Received an empty message.");
    }
    else
    {
      ROS_INFO("Got an element. Trying to center it.");
    }

   
//Determining the lowest corner of the element by finding the point with the maximum x coordinate; Then taking into account the distances to the next corner points, so that the element can be brought into the horizontal position



    for(int p = 2; p < elem.size()+2; p++)
    {
      if(elem.at(p%4).y > max_y)
      {
        max_y = elem.at(p%4).y;

        
//Detection of the two Euclidean distances between the point and the two nearest points


        double first_distance = cv::norm(cv::Mat(elem.at(p%4)), cv::Mat(elem.at((p-1)%4)));
        double second_distance = cv::norm(cv::Mat(elem.at(p%4)), cv::Mat(elem.at((p+1)%4)));

        
//Determine the corners at the bottom left and top right using the previously calculated distances and definition of a horizontal target image



        if(elem.at((p-1)%4).x > elem.at((p+1)%4).x)
        {
          if(first_distance > second_distance)
          {
            bottom_right = (p-1)%4;
            bottom_left = p%4;
            top_right = p-2;
            dst = 255 * cv::Mat::ones(cvRound(second_distance), cvRound(first_distance), src.type());
          }
          else
          {
            bottom_right = p%4;
            bottom_left = (p+1)%4;
            top_right = (p-1)%4;
            dst = 255 * cv::Mat::ones(cvRound(first_distance), cvRound(second_distance), src.type());
          }
        }
        else
        {
          if(first_distance > second_distance)
          {
            bottom_right = p%4;
            bottom_left = (p-1)%4;
            top_right = (p+1)%4;
            dst = 255 * cv::Mat::ones(cvRound(second_distance), cvRound(first_distance), src.type());
          }
          else
          {
            bottom_right = (p+1)%4;
            bottom_left = p%4;
            top_right = (p+2)%4;
            dst = 255 * cv::Mat::ones(cvRound(first_distance), cvRound(second_distance), src.type());
          }
        }
      }
    }

    
//Determination of the 2x3 transformation matrix by specifying the change in the localization of the three corner pixels


    cv::Point2f src_pt_tri[3] = {elem.at(bottom_left), elem.at(bottom_right), elem.at(top_right)};
    cv::Point2f dst_pt_tri[3] = {cv::Point2f(0, dst.rows - 1), cv::Point2f(dst.cols - 1, dst.rows - 1), cv::Point2f(dst.cols - 1, 0)};
    transform_matrix = cv::getAffineTransform(src_pt_tri, dst_pt_tri);

    
//Application of the affine transformation with the determined transformation matrix to the source image in order to obtain the target image


    cv::warpAffine(src, dst, transform_matrix, dst.size());

    
//Display and save the resulting image


    cv::imwrite("/home/ros/Bilder/centered_image.png", dst);
    cv::imshow("centered image", dst);
    cv::waitKey(0);

    
//Packing the target image in ROS image format; If necessary, output of an error message during conversion



    cv_dst_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, dst);
    try
    {
      ros_dst_img_ptr = cv_dst_img.toImageMsg();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
    }

   
//Publish the image


    img_pub_.publish(ros_dst_img_ptr);
    ROS_INFO("Published the centered image.");
  }
};

int main(int argc, char** argv)
{
  
// Node build up


  ros::init(argc, argv, "deduce_transform_matrix");

  
//Creation of an object of the class created for the execution of the code


  Centering c;

  
//Wait for callbacks by received messages, as long as the node is not switched off


  while(ros::ok())
  {
    ROS_INFO_ONCE("Waiting for advertised images and topics on the subscribed topics.");
    ros::spinOnce();
  }

  return 0;
}
