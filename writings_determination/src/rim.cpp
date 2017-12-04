#include <ros/ros.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <writings_determination/image_exchange.h>

//Function to remove possible errors at the edge of a grayscale image, which shows a background by placing corresponding pixel values on the background color
void blurRim(const cv::Mat& src, cv::Mat& dst)
{
  cv::Mat img, background_mask, first_background_mask, second_background_mask;
  cv::Scalar_<double> mean, std, background_mean;
  
  int i, j;
  
  //Calculation of mean value and standard deviation of all pixel values of the source image
  
  src.copyTo(img);
  cv::meanStdDev(img, mean, std);
  
  cv::imshow("test image", img);
  cv::waitKey(0);
  ROS_INFO("OpenCV-Versionsnummer: %s", CV_VERSION);
  ROS_INFO("Average: %ld", (long int) mean[0]);
  ROS_INFO("standard deviation: %ld",(long int) std[0]);
  
  
  //Generating a binary matrix indicating for the pixel values whether they are within the 0.75-sigma distance to the arithmetic mean; Median filters to compensate for individual pixel inaccuracies
  
  first_background_mask = (cv::abs(src - mean[0]) < 0.75*std[0]);
  cv::medianBlur(first_background_mask, second_background_mask, 5);
  cv::bitwise_and(first_background_mask, second_background_mask, background_mask);
  
  //Calculation of the arithmetic mean of all pixel values in the addressed 0.75 sigma interval
  
  background_mean = cv::mean(src, background_mask);
  
  //Initialize Boolean change matrix with 0
  
  cv::Mat_<bool> change = cv::Mat_<bool>::zeros(img.size());
  
  ROS_INFO("src.cols = %d   src.rows = %d", src.cols , src.rows);
  
  
  //All values of the points marked in the change matrix are stored to the calculated mean value of the 0.75 sigma interval
  
  img.copyTo(dst);
  dst = img.setTo(background_mean, change);
  
  int iN = 10;
  
  for(int y=0;y<iN;y++)
  {
    for(int x=0;x<dst.cols;x++)
    {
      // set pixel
      dst.at<char>(cv::Point(x,y)) = 255;
      dst.at<char>(cv::Point(x,dst.rows-y-1)) = 255;
    }
  }
  
  for(int y=0;y<dst.rows;y++)
  {
    for(int x=0;x<iN;x++)
    {
      // set pixel
      dst.at<char>(cv::Point(x,y)) = 255;
      dst.at<char>(cv::Point(dst.cols-x-1,y)) = 255;
    }
  }
}

//Steps to follow after the service request from binarization

bool blurRimCallback(writings_determination::image_exchange::Request& req, writings_determination::image_exchange::Response& res)
{
  sensor_msgs::Image ros_src_img;
  cv_bridge::CvImagePtr cv_src_img_ptr;
  cv::Mat src, dst;
  cv_bridge::CvImage cv_dst_img;
  sensor_msgs::ImagePtr ros_dst_img_ptr;
  
  //Removing the source image from the request
  ros_src_img = req.img;
  
  //With error message secure attempt to translate the image content of the message into OpenCV-compatible format
  try
  {
    cv_src_img_ptr = cv_bridge::toCvCopy(ros_src_img, std::string());
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
    return false;
  }
  
  //Caching the source image under a different name
  src = cv_src_img_ptr->image;
  
  //If the pixels are composed of color triplets, a gray conversion is carried out before the inspection of the edge
  if(src.channels() == 3)
  {
    cv::cvtColor(src, src, CV_BGR2GRAY);
  }
  
  
  //morphological operation for obtaining appropriate thickness 
  
  cv::Mat dst1,output,dst2,dst3;
  
  cv::adaptiveThreshold(src,dst1,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,21,2);
  
  cv::Mat se1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2));
  
  cv::dilate(dst1,dst2,se1);
  
  float erosion_size = 1.5;
  
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),cv::Point(erosion_size, erosion_size) );
  
  
  
  
  cv::erode(dst2,dst3,element);

 //Reducing salt and pepper noise
   
  cv::dilate(dst3,dst3,se1);
  cv::medianBlur(dst3,output,5);
  
  //cropped image to remove extra boundaries

int offset_x = 30;
int offset_y = 20;

cv::Rect roi;
roi.x = offset_x;
roi.y = offset_y;
roi.width = output.size().width - (offset_x*2);
roi.height = output.size().height - (offset_y*2);

cv::Mat crop = output(roi);



  
  //Examination of the photographic background for inconsistencies
  blurRim(crop, dst);   //input of the function is "output" to this function
  
  //Visualize the result and leave it in a file for later purposes
  cv::imwrite("/home/ros/Bilder/blurred_rim.png", dst);
  cv::imshow("gray image with blurred rim", dst);
  cv::waitKey(0);
  
  
  
  //Transfer of the target image to the ROS image format
  cv_dst_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, dst);//sending diluted image
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

int main(int argc, char** argv)
{
  //Initialization of the node
  ros::init(argc, argv, "rim");
  
  ros::NodeHandle node;
  ros::ServiceServer blur_rim_server;
  
  //Service of the binarization, whereby the limit value used can be accessed via parameter servers
  blur_rim_server = node.advertiseService("blur_rim", blurRimCallback);
  
  //Waiting for callbacks to be triggered by received service requests
  ROS_INFO("Ready to react on service requests for 'blur_rim'.");
  ros::spin();
  
  return 0;
}

