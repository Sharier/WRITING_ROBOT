
//Include the libraries, including the required srv files, but also the necessary APIs for converting OpenCV and ROS images


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <writings_determination/image_exchange.h>
#include <writings_determination/image_check.h>


//Notation of the default setting of the limit value and the possible inversion after which the binary decision is made


const double DEFAULT_THRESH = 200.;
const int DEFAULT_TYPE = cv::THRESH_BINARY_INV;


//Function which expects an image 8-bit channels as an input parameter, which is tested with respect to the present binary


bool isBinary(const cv::Mat* const img_ptr)
{
  
//If there are multiple channels, this is not a binary image and false can be returned


  if((*img_ptr).channels() != 1)
  {
    return false;
  }
  
//For only one channel, check that the pixels are only occupied with values from [0,1]


  else
  {
    cv::MatND hist;

   
//Determination of the bins

    int num_bins = 2;
    const float bin_boundaries[3] = {0.f, 2.f, 256.f};
    const float* first_boundary_ptr = bin_boundaries;

  
//Calculation of the histogram for the two bins [0,1] and [2,255]


    cv::calcHist(img_ptr, 1, 0, cv::Mat(), hist, 1, &num_bins, &first_boundary_ptr, false, false);

   
//Query whether pixel values lie in the larger of the two bins. If this is not the case, the image is binary


    if(hist.at<float>(1) == 0.f)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}


//Function to convert an image using threshold decisions to a binary image


void binarization(const cv::Mat& src, cv::Mat& dst, const double& thresh, const int& type = cv::THRESH_BINARY_INV)
{
 
//If the image is already binary, do not make any changes


  if(isBinary(&src))
  {
    dst = src;
  }
  
//In a black-and-white image, only the limit decision is necessary for transfer to the binary image


  else
  {
    cv::threshold(src, dst, thresh, 1, type);
  }
}


//Steps to follow after the service request from binarization


bool binarizationCallback(writings_determination::image_exchange::Request& req, writings_determination::image_exchange::Response& res)
{
  sensor_msgs::Image ros_src_img;
  cv_bridge::CvImagePtr cv_src_img_ptr;
  cv::Mat dst;
  cv_bridge::CvImage cv_dst_img;
  sensor_msgs::ImagePtr ros_dst_img_ptr;
  double thresh;
  int type;

 
//Removing the source image from the request


  ros_src_img = req.img;

 
//With error message secure attempt to translate the image content of the message into OpenCV-compatible format


  try
  {
    cv_src_img_ptr = cv_bridge::toCvCopy(ros_src_img, std::string());
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
    return false;
  }

 
//Update the variables corresponding to the ROS parameters


  thresh = ros::param::param("~threshold", DEFAULT_THRESH);
  type = ros::param::param("~type", DEFAULT_TYPE);

 
//Carry out the binarization on the received photo


  binarization(cv_src_img_ptr->image, dst, thresh, type);

  
//Display and save the result of the binarization


  cv::Mat show_dst;
  cv::threshold(dst, show_dst, 0.5, 255., cv::THRESH_BINARY_INV);
  cv::imwrite("/home/ros/Bilder/binary_image.png", show_dst);
  cv::imshow("binary image", show_dst);
  cv::waitKey(0);


//Transfer of the target image to the ROS image format


  cv_dst_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, dst);
  try
  {
    ros_dst_img_ptr = cv_dst_img.toImageMsg();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Problem with cv_bridge regarding the conversion from CV to ROS format: %s", e.what());
    return false;
  }

  
//Integrate the target image into the response


  res.img = *ros_dst_img_ptr;

  
//Informing the client about the successful execution of the service 'binarization'


  return true;
}


//Steps to follow after the is_binary service request


bool isBinaryCallback(writings_determination::image_check::Request& req, writings_determination::image_check::Response& res)
{
  sensor_msgs::Image ros_img;
  cv_bridge::CvImageConstPtr cv_img_ptr;
  cv::Mat img;

 
//Creation of a shared_ptr on the ROS image


  boost::shared_ptr<sensor_msgs::Image> ros_img_ptr;
  *ros_img_ptr = req.img;

 
//With error message secure attempt to translate the image content of the message into OpenCV-compatible format


  try
  {
    
//Use of toCvShare, since the image must not be changed, but only checked


    cv_img_ptr = cv_bridge::toCvShare(ros_img_ptr);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
    return false;
  }

 
//Test whether image is binary, with saving the result in the response


  res.logical_value = isBinary(&(cv_img_ptr->image));

 
//Informing the client about the successful implementation of the service 'is_binary'


  return true;
}

int main(int argc, char** argv)
{
  
//Initialization of the node


  ros::init(argc, argv, "binary");

  ros::NodeHandle node;
  ros::ServiceServer binarization_server;
  ros::ServiceServer is_binary_server;

  
//Service of the binarization, whereby the limit value used can be accessed via parameter servers


  binarization_server = node.advertiseService("binarization", binarizationCallback);

 
//Generation and initialization of the parameters for limit value and inversion, if these are not yet available
  if(!ros::param::has("~threshold"))
  {
    ros::param::set("~threshold", DEFAULT_THRESH);
  }
  if(!ros::param::has("~type"))
  {
    ros::param::set("~type", DEFAULT_TYPE);
  }

  
//Service 'is_binary' about the ROS MASTER public


  is_binary_server = node.advertiseService("is_binary", isBinaryCallback);


//Waiting for callbacks to be triggered by received service requests


  ROS_INFO("Ready to react on service requests for 'binarization' and 'is_binary'.");
  ros::spin();

  
//Remove special parameters for this node from the server


  ros::param::del("~threshold");
  ros::param::del("~type");

  return 0;
}
