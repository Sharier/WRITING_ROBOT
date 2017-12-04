#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <writings_determination/image_exchange.h>
using namespace std;

const int FIRST_SUBITERATION = 0;
const int SECOND_SUBITERATION = 1;
const int condition2= 2;

// Function that contains the iterative steps of the implemented algorithm, with two different partial iterations
bool subiteration(cv::Mat& img, const int& type)
{

  int first_term, second_term, third_term, fourth_term, fifth_term,sixth_term,seventh_term,eigth_term,ninth_term,tenth_term;

  // For easier access to pixels, a Mat_ object is used
  cv::Mat_<unsigned char> kernel = cv::Mat_<unsigned char>(3,3);

  // Initialize inversion matrix with zeros
  cv::Mat invert = cv::Mat::zeros(img.rows, img.cols, CV_8U);

 // Check each pixel individually for conditions
  for(int i = 1; i < img.rows-1; i++)
  {
    for(int j = 1; j < img.cols-1; j++)
    {
     // Extract matrix with current element and direct neighboring elements
      kernel = img(cv::Range(i-1, i+2), cv::Range(j-1, j+2));

      // initial requirement
      if(kernel(1,1) == 1)
      {
         if(type==0)
          {
              first_term = cv::sum(kernel)[0];
              second_term = (kernel(0, 0) == 0 && kernel(0, 1) == 1) + (kernel(0, 1) == 0 && kernel(0, 2) == 1) +
                      (kernel(0, 2) == 0 && kernel(1, 2) == 1) + (kernel(1, 2) == 0 && kernel(2, 2) == 1) +
                      (kernel(2, 2) == 0 && kernel(2, 1) == 1) + (kernel(2, 1) == 0 && kernel(2, 0) == 1) +
                      (kernel(2, 0) == 0 && kernel(1, 0) == 1) + (kernel(1, 0) == 0 && kernel(0, 0) == 1);
              third_term=(kernel(0, 1) * kernel(1, 2) * kernel(2, 1));
              fourth_term=(kernel(1, 0) * kernel(1, 2)) * kernel(2, 1);
              if(first_term >=3 && first_term <=7 && second_term == 1 && third_term == 0 && fourth_term == 0)
              {
                  invert.at<unsigned char>(i, j) = 1;
              }
          }
          else if(type==1)
          {
              first_term = cv::sum(kernel)[0];
              second_term = (kernel(0, 0) == 0 && kernel(0, 1) == 1) + (kernel(0, 1) == 0 && kernel(0, 2) == 1) +
                      (kernel(0, 2) == 0 && kernel(1, 2) == 1) + (kernel(1, 2) == 0 && kernel(2, 2) == 1) +
                      (kernel(2, 2) == 0 && kernel(2, 1) == 1) + (kernel(2, 1) == 0 && kernel(2, 0) == 1) +
                      (kernel(2, 0) == 0 && kernel(1, 0) == 1) + (kernel(1, 0) == 0 && kernel(0, 0) == 1);
              third_term = (kernel(0, 1) * kernel(1, 2) * kernel(1, 0));
              fourth_term = (kernel(0, 1) * kernel(2, 1) * kernel(1, 0));
              if(first_term >=4 && first_term <=7 && second_term == 1 && third_term == 0 && fourth_term == 0)
              {
                  invert.at<unsigned char>(i, j) = 1;
              }
          }

          else
          {
              fifth_term= (kernel(0, 0) * kernel(1, 0) * kernel(2, 1));//number 1
              sixth_term= kernel(0,2);//p3
              seventh_term=(kernel(0, 2) * kernel(1, 2) * kernel(2, 1));//number 2
              eigth_term=(kernel(2, 2) * kernel(2, 1) * kernel(1, 0));//number 3
              ninth_term=kernel(0,0);//p1
              tenth_term=(kernel(1, 2) * kernel(2, 1) * kernel(2, 0));//number 4
              if(fifth_term==1 && sixth_term ==0 && seventh_term == 1 && ninth_term == 0 && eigth_term == 1 && sixth_term ==0 && tenth_term==1 && ninth_term == 0 )
              {
                  invert.at<unsigned char>(i, j) = 1;
              }


          }



      }
    }
}
// inverting all the pixels that hit the conditions by subtracting the inverse matrix, if different from 0; Return if the subiteration has caused a change

  if(cv::countNonZero(invert) == 0)
  {

    return 0;
  }
  else
  {
    img = img - invert;
    return 1;
  }
}

//Function for applying the improved algorithm

void skeletonization(const cv::Mat& src, cv::Mat& dst)
{
  cv::Mat iteration_result;

  //Change variables as true
  bool change = true;

 //Expansion of the obtained source image around an outer layer of the width so that neighboring pixels also exist for the outer pixels of the input image
  cv::copyMakeBorder(src, iteration_result, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar_<unsigned char>(0));

//Repeat iterative steps as long as changes can occur
  while(change == true)
  {
    //Carry out iterative steps
    change = subiteration(iteration_result, FIRST_SUBITERATION);
    change = (subiteration(iteration_result, SECOND_SUBITERATION) || change);
    change = (subiteration(iteration_result, condition2) || change);
  }

  //Output image to the end result of the iterations
  dst = iteration_result(cv::Range(1, iteration_result.rows-1), cv::Range(1, iteration_result.cols-1));
}

//Callback function, which controls the steps to be executed when a service is requested
bool rxCallback(writings_determination::image_exchange::Request& req, writings_determination::image_exchange::Response& res)
{
  sensor_msgs::Image ros_src_img;
  cv_bridge::CvImagePtr cv_src_img_ptr;
  cv::Mat dst;
  cv_bridge::CvImage cv_dst_img;
  sensor_msgs::ImagePtr ros_dst_img_ptr;

//Removing the source image from the request
  ros_src_img = req.img;

//With error messages secured attempt to translate the image content of the message into OpenCV-compatible format, differentiation according to one and three channels
  try
  {
    if(sensor_msgs::image_encodings::isColor(ros_src_img.encoding))
    {
      ROS_ERROR("Need a binary image for skeletonization where 1 stands for writings.");
    }
    else
    {
      cv_src_img_ptr = cv_bridge::toCvCopy(ros_src_img, sensor_msgs::image_encodings::MONO8);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
    return false;
  }

 //Skeleton of the OpenCV image
  skeletonization(cv_src_img_ptr->image, dst);

//Skeleton and store the corresponding image
  cv::Mat show_dst;
  cv::threshold(dst, show_dst, 0.5, 255., cv::THRESH_BINARY_INV);
  cv::imwrite("/home/ros/Bilder/skeleton_image.png", show_dst);
  cv::imshow("skeleton image", show_dst);
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

//Confirmation of the successful execution of the service
  return true;
}

int main(int argc, char** argv)
{
 //Initialization of the node
  ros::init(argc, argv, "skeleton");

//Creation of handles for node and server
  ros::NodeHandle node;
  ros::ServiceServer server;

//Offer of the service 'skeletonization'
  server = node.advertiseService("skeletonization", rxCallback);

  //Waiting for callbacks to be triggered by received messages
  ROS_INFO("Ready to react on service requests for 'skeletonization'.");
  ros::spin();

  return 0;
}
