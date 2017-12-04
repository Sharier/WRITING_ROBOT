
//Including the libraries, including interfaces for the exchange of images


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <writings_determination/image_exchange.h>


//Class for processing an image of a sheet of paper which serves the clear identification of writing on it


class WritingsDeterminer
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::ServiceClient blur_rim_client_;
  ros::ServiceClient binarization_client_;
  ros::ServiceClient skeletonization_client_;
  image_transport::Publisher img_pub_;

public:
  WritingsDeterminer()
    : it_(nh_)
  {
    
//Definition of topics from / on which subscribed respectively advertised


    img_sub_ = it_.subscribe("image/paper/raw", 1, &WritingsDeterminer::rxCallback, this);
    img_pub_ = it_.advertise("image/paper/writings", 1);

    
//Determination of the services that are claimed over the various objects


    blur_rim_client_ = nh_.serviceClient<writings_determination::image_exchange>("blur_rim");
    binarization_client_ = nh_.serviceClient<writings_determination::image_exchange>("binarization");
    skeletonization_client_ = nh_.serviceClient<writings_determination::image_exchange>("skeletonization");
  }

 
//Callback function is always called when messages appear on the subscribed topic, thus regulating the processing of the images thus obtained



  void rxCallback(const sensor_msgs::ImageConstPtr& ros_src_img_ptr)
  {
    writings_determination::image_exchange srv_blur_rim;
    writings_determination::image_exchange srv_binarization;
    writings_determination::image_exchange srv_skeletonization;

    
//Use of the service blur_rim


    srv_blur_rim.request.img = *ros_src_img_ptr;
    if(blur_rim_client_.call(srv_blur_rim))
    {
    
//Use of the service binarization


      srv_binarization.request.img = srv_blur_rim.response.img;
      if(binarization_client_.call(srv_binarization))
      {
        
//Utilizing the service skeletonization, if the binarization was previously successful



        srv_skeletonization.request.img = srv_binarization.response.img;
        if(skeletonization_client_.call(srv_skeletonization))
        {
     
//Publish the image obtained through service use


          img_pub_.publish(srv_skeletonization.response.img);
          ROS_INFO("Scanned and processed a picture for writing lines. Publishing the resulting picture on a new topic.");
        }
        else
        {
          ROS_ERROR("Failed to call service 'skeletonization'.");
        }
      }
      else
      {
        ROS_ERROR("Failed to call service 'binarization'.");
      }
    }
    else
    {
      ROS_ERROR("Failed to call service 'blur_rim'.");
    }
  }
};

int main(int argc, char** argv)
{
  
//Node


  ros::init(argc, argv, "writings_determination");

 
//Object of the class in which the actual steps are defined


  WritingsDeterminer wd;

  
//Waiting until messages arrive for which callbacks are triggered


  ROS_INFO("Waiting for paper pictures to examine them for writings.");
  ros::spin();

  return 0;
}
