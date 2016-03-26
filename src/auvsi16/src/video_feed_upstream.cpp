#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;

double resize_width_factor = 0.5;
double resize_height_factor = 0.5;
image_transport::Publisher pub_video_resize;
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_feed_upstream"); // Nama program: video_listener
  ros::NodeHandle nh("~");
  
  nh.getParam("resize_width_factor", resize_width_factor);
  nh.getParam("resize_height_factor", resize_height_factor);
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/auvsi16/video", 1, imageCallback); // Topik subscribe: auvsi16/video
  pub_video_resize = it.advertise("/auvsi16/video/resize", 1, true); 	// topic: auvsi16/video; Queue: 1; Latch :true
  
  ros::spin();

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image_received;  
  cv::Mat resized_frame;
  sensor_msgs::ImagePtr image_msg_resize;

  try
  {
	image_received  = cv_bridge::toCvCopy(msg, "bgr8")->image;		// use cv_bridge::toCvCopy instead of cv_bridge::toCvShare to process a mat data
    
    resize(image_received, resized_frame, Size(), resize_width_factor, resize_height_factor, INTER_LINEAR);
	image_msg_resize = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_frame).toImageMsg();
	pub_video_resize.publish(image_msg_resize);
  }
  
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
