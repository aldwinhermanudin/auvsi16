#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/CompressedImage.h>

using namespace cv;
double resize_width_factor = 0.5;
double resize_height_factor = 0.5;
vector<int> param = vector<int>(2);      
ros::Publisher pub_video_front_resize;
ros::Publisher pub_video_right_resize;
void imageFrontCallback(const sensor_msgs::ImageConstPtr& msg);
void imageRightCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv){
	
  ros::init(argc, argv, "video_feed_upstream"); // Nama program: video_listener
  ros::NodeHandle nh("~");
  
  param[0] = CV_IMWRITE_JPEG_QUALITY;			// set tipe encoding 
  param[1] = 30;								// set kualitas encoding
  
  nh.getParam("resize_width_factor", resize_width_factor);
  nh.getParam("resize_height_factor", resize_height_factor);
  nh.getParam("jpeg_quality", param[1]);
    
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_front = it.subscribe("/auvsi16/video/front", 1, imageFrontCallback); // Topik subscribe: auvsi16/video
  pub_video_front_resize = nh.advertise<sensor_msgs::CompressedImage>("/auvsi16/video/front/upstream", 1, true);
  
  image_transport::Subscriber sub_right = it.subscribe("/auvsi16/video/right", 1, imageRightCallback); // Topik subscribe: auvsi16/video
  pub_video_front_resize = nh.advertise<sensor_msgs::CompressedImage>("/auvsi16/video/right/upstream", 1, true);
  
  ros::spin();
}

void imageFrontCallback(const sensor_msgs::ImageConstPtr& msg){
	
	cv_bridge::CvImagePtr cv_ptr;
	try{
	
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		resize(cv_ptr->image, cv_ptr->image, Size(), resize_width_factor, resize_height_factor, INTER_LINEAR);
		vector<uchar> buff;
		imencode(".jpeg",cv_ptr->image, buff,param);
		sensor_msgs::CompressedImage buff_message;
		buff_message.data = buff;
		pub_video_front_resize.publish(buff_message);
	}
  
  catch (cv_bridge::Exception& e){
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageRightCallback(const sensor_msgs::ImageConstPtr& msg){
	
  cv_bridge::CvImagePtr cv_ptr;
  try{
	
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		resize(cv_ptr->image, cv_ptr->image, Size(), resize_width_factor, resize_height_factor, INTER_LINEAR);
		vector<uchar> buff;
		imencode(".jpeg",cv_ptr->image, buff,param);
		sensor_msgs::CompressedImage buff_message;
		buff_message.data = buff;
		pub_video_right_resize.publish(buff_message);
  }
  
  catch (cv_bridge::Exception& e){
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
