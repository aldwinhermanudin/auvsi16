#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <iostream>

using namespace std;

void imageCallback(const sensor_msgs::CompressedImage& msg)
{
	cv::Mat img_stream = cv::imdecode(cv::Mat(msg.data),1);
	if (!img_stream.empty()){
		cv::imshow("Front Camera", img_stream);
	}
		
	if (cv::waitKey(30) == 27){
		cout << "esc key is pressed by user" << endl;
		ros::shutdown();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosub_monitor");
  ros::NodeHandle nh;
  cv::namedWindow("Front Camera", cv::WINDOW_NORMAL);
  cv::startWindowThread();
  ros::Subscriber sub_front = nh.subscribe("auvsi16/robosub/video/front", 1, imageCallback,ros::TransportHints().unreliable()); // setting queue to 1 remove delay in compressedImage data | also using UDP
  ros::spin();
  cv::destroyWindow("Front Camera");
  }
