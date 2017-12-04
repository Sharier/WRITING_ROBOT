
//Including the libraries, including empty service type and transport of images in ROS


#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/chrono/chrono.hpp>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "environment";


//Class which takes pictures of a topic and always publishes the current picture to another topic when the offered service is requested; Image stream is also displayed in windows


class ImageDiverter
{
private:
  ros::NodeHandle node_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
  boost::mutex lock_;
  bool update_image_;
  bool running_;
  const int queue_size_;
  bool let_pass_;
  cv::Mat img_;
  ros::AsyncSpinner spinner_;
  ros::ServiceServer server_;

  
//Function, which includes the action steps in response to received images


  void rxImageCallback(const sensor_msgs::ImageConstPtr& ros_img_ptr)
  {
    cv::Mat img;

    
//Photo in OpenCV


    if(ros_img_ptr == NULL)
    {
      ROS_ERROR("Received no image data.");
      return;
    }
    else
    {
      readImage(ros_img_ptr, img);
    }

    
//Save image as member and display a new image via customized variable


    lock_.lock();
    this->img_ = img;
    update_image_ = true;
    lock_.unlock();
  }

  
//Callback function, which provides the service to enable the passage of an image


  bool enableRedirecting(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
   
//Allow an incoming image to be submitted



    lock_.lock();
    let_pass_ = true;
    lock_.unlock();

   
//Returning true represents the successful application of the service



    return 1;
  }

  
//Function to display the current image


  void imageViewer()
  {
    cv::Mat img;

   
//Publisher create 


    pub_ = it_.advertise("image/raw/environment", 1);

    




    
//Endless loop that ends only if the end of the action within the class is required



    while(running_ && ros::ok())
    {
      
//If a logical variable indicates that a new image is present, this is updated in the output window and the logical variable is inverted




      if(update_image_)
      {
        lock_.lock();
        img = this->img_;
        update_image_ = false;
        lock_.unlock();
        //cv::imshow(OPENCV_WINDOW, img);


       
//If a logical member variable indicates that the image is to be forwarded, the function for publishing images on another topic is called




        if(let_pass_)
        {
          imagePublisher();
        }
      }
    }

    
//Close the OpenCV window


    cv::destroyAllWindows();
  }

  
//Function, which publishes last image on the new topic



  void imagePublisher()
  {
    sensor_msgs::ImagePtr ros_img_ptr;
    cv_bridge::CvImage cv_img;
    cv::Mat img;

   
//Packing the target image in ROS image format; If necessary, output of an error message during conversion




    lock_.lock();
    img = this->img_;
    lock_.unlock();
    cv_img = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, img);
    try
    {
      ros_img_ptr = cv_img.toImageMsg();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
      return;
    }

    
//Publish the image


    ros_img_ptr->header.stamp = ros::Time::now();
    pub_.publish(ros_img_ptr);
    ROS_INFO("Redirected an image successfully.");

    
//Set the passier variable back to false so that no further photos are redirected



    lock_.lock();
    let_pass_ = false;
    lock_.unlock();
  }

 
//Convert the incoming image to the more manageable OpenCV format Mat



  void readImage(const sensor_msgs::Image::ConstPtr ros_img_ptr, cv::Mat &img) const
  {
    cv_bridge::CvImageConstPtr cv_img_ptr;

   
//Transfer of the received ROS internal image to OpenCV-compatible image of the format BGR8, if this project fails


  
//An ROS error message


    try
    {
      cv_img_ptr = cv_bridge::toCvShare(ros_img_ptr, sensor_msgs::image_encodings::BGR8); //cv_img_ptr->encoding

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Problem with cv_bridge regarding the conversion from ROS to CV format: %s", e.what());
      return;
    }

   
//Store the image data in a given argument


    cv_img_ptr->image.copyTo(img);
  }

public:
  ImageDiverter()
    : update_image_(false), running_(false), queue_size_(5), let_pass_(false), spinner_(0), it_(node_)
  {
  }

  ~ImageDiverter()
  {
  }

  
//Function to run the actions included in the class


  void run()
  {
    
//Variable to indicate that the class is active


    running_ = true;

    
//Setting up a subscriber unit for receiving images


    sub_ = it_.subscribe("/kinect2/hd/image_color_rect", queue_size_, &ImageDiverter::rxImageCallback, this);

   
//Notation of the offered service


    server_ = node_.advertiseService("redirect_image", &ImageDiverter::enableRedirecting, this);
    ROS_INFO("Waiting for demands for service 'redirect_image'.");

    
//Enables callbacks in different threads


    spinner_.start();

   
//Periodically check whether the image can be displayed and the node does not need to be down



    boost::chrono::milliseconds duration(1);
    while(!update_image_)
    {
      if(!ros::ok())
      {
        return;
      }
      boost::this_thread::sleep_for(duration);
    }

    
//Call the function to display



    imageViewer();

    
//Stop parallel execution



    spinner_.stop();

  
//Class no longer runs, so adjust corresponding variable




    running_ = false;
  }
};

int main(int argc, char **argv)
{
  
//Activation of the node to use the ROS features



  ros::init(argc, argv, "starting_node");

 
//If there are grounds for the ending of the action, this is done



  if(!ros::ok())
  {
    return 0;
  }

  
//Class to display and forward the images


  ImageDiverter id;
  id.run();

  
//Close the ROS node


  ros::shutdown();

  return 0;
}
