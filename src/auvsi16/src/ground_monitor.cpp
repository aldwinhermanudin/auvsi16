#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <iostream>

using namespace std;
cv::Mat img_stream ;

void imageCallback(const sensor_msgs::CompressedImage& msg)
{
	img_stream = cv::imdecode(cv::Mat(msg.data),1);
	if (!img_stream.empty()){
		cv::imshow("Compressed Video", img_stream);
	}
		
	if (cv::waitKey(30) == 27){
		cout << "esc key is pressed by user" << endl;
		ros::shutdown();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_monitor");
  ros::NodeHandle nh;
  cv::namedWindow("Compressed Video", cv::WINDOW_NORMAL);
  cv::startWindowThread();
  ros::Subscriber sub = nh.subscribe("auvsi16/video/resize/compressed", 1, imageCallback,ros::TransportHints().unreliable()); // setting queue to 1 remove delay in compressedImage data | also using UDP
  ros::spin();
  cv::destroyWindow("Compressed Video");
}
