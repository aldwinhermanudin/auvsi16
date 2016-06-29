// ########################################## start of MavrosInterface class ########################################## //
#include "ros/ros.h"
#include "auvsi16/HSVValue.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <string>

using namespace std;

class ImageProcessingInterface {

protected:

  ros::NodeHandle nh;
  ros::Publisher 	pub_imgproc_select;
  ros::Publisher 	pub_hsv_value;

public:
  ImageProcessingInterface  ();
  void configuration(int conf);
  void setHSVRange	(int lowh, int highh, int lows, int highs, int lowv, int highv);
};

ImageProcessingInterface::ImageProcessingInterface(){
  pub_hsv_value	= nh.advertise<auvsi16::HSVValue>("/auvsi16/node/image_processing/hsv", 16,true);
	pub_imgproc_select	= nh.advertise<std_msgs::Int16>("/auvsi16/node/image_processing/select", 16,true);

  }

void ImageProcessingInterface::configuration(int conf){
  	std_msgs::Int16 imgproc_select;

  	imgproc_select.data = conf;
  	pub_imgproc_select.publish(imgproc_select);
  	ROS_WARN_STREAM("Changing Image Processing to " << conf);
  	sleep(1);
  }

  void 	ImageProcessingInterface::setHSVRange	(int lowh, int highh, int lows, int highs, int lowv, int highv){
  	auvsi16::HSVValue hsv_value;
  	hsv_value.iLowH 		= lowh;	// yellow = 15, red = 166 , blue = 92
  	hsv_value.iHighH 		= highh;	// yellow = 29, red = 179 , blue = 114
  	hsv_value.iLowS			= lows; // lower this to allow more noise, 80 is normal
  	hsv_value.iHighS 		= highs;
  	hsv_value.iLowV 		= lowv; // lower this to allow more noise
  	hsv_value.iHighV 		= highv;
  	pub_hsv_value.publish(hsv_value);
  	ROS_WARN_STREAM("HSV Value changed");

  }

// ########################################## start of MavrosInterface class ########################################## //
