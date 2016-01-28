/********************************************************************************************************************
 * Program name	:	video_listener																					*
 * Last editor	:	Arief Purnama Muharram																			*
 * Reference	:	http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages								*	
 * Reference	:	http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages	*
 * *****************************************************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

cv::Mat image_received;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
	image_received  = cv_bridge::toCvCopy(msg, "bgr8")->image;		// use cv_bridge::toCvCopy instead of cv_bridge::toCvShare to process a mat data
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  //example of modifing a mat variable after cv_bridge::toCvCopy
  cv::circle(image_received, cv::Point(50, 50), 10, CV_RGB(255,0,0));
  
  cv::imshow("view", image_received);
  
  //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, exit node
  if(cv::waitKey(30) == 27){ 
    std::cout << "esc key is pressed by user" << std::endl; 
	ros::shutdown();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_listener"); // Nama program: video_listener
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("auvsi16/video", 1, imageCallback); // Topik subscribe: auvsi16/video
  ros::spin();
  cv::destroyWindow("view");
}
