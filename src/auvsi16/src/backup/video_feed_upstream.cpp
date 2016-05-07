#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

// Update this code using the following reference : http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

using namespace cv;
double resize_width_factor = 0.5;
double resize_height_factor = 0.5;
image_transport::Publisher pub_video_resize;
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv){
	
  ros::init(argc, argv, "video_feed_upstream"); // Nama program: video_listener
  ros::NodeHandle nh("~");
  
  nh.getParam("resize_width_factor", resize_width_factor);
  nh.getParam("resize_height_factor", resize_height_factor);
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/auvsi16/video/front", 1, imageCallback); // Topik subscribe: auvsi16/video
  pub_video_resize = it.advertise("/auvsi16/video/front/resize", 1, true); 	// topic: auvsi16/video; Queue: 1; Latch :true
  
  ros::spin();

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
	
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	
    resize(cv_ptr->image, cv_ptr->image, Size(), resize_width_factor, resize_height_factor, INTER_LINEAR);
	pub_video_resize.publish(cv_ptr->toImageMsg());
  }
  
  catch (cv_bridge::Exception& e){
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
