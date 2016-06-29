#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <iostream>

using namespace std;
using namespace cv;

int iLowH 	= 15;	// yellow = 15, red = 166
int iHighH 	= 29;	// yellow = 29, red = 179
	
int iLowS	= 90; // lower this to allow more noise, 80 is normal
int iHighS 	= 184;

int iLowV 	= 138; // lower this to allow more noise
int iHighV 	= 255;

ros::Publisher pub_control;
std_msgs::Int32MultiArray control_data; 

void enableControl(){
	
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);
	
	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);
	
	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);
}


void controlDataTransmit(){
	
	control_data.data.clear();
	control_data.data.push_back(iLowH);
	control_data.data.push_back(iHighH);
	control_data.data.push_back(iLowS);
	control_data.data.push_back(iHighS);
	control_data.data.push_back(iLowV);
	control_data.data.push_back(iHighV);
	pub_control.publish(control_data);
}

void imageOriginalCallback(const sensor_msgs::CompressedImage& msg)
{
	controlDataTransmit();
	Mat img_stream;
	img_stream = cv::imdecode(cv::Mat(msg.data),1);
	if (!img_stream.empty()){
		cv::imshow("Compressed Video Original", img_stream);
	}
		
	if (cv::waitKey(30) == 27){
		cout << "esc key is pressed by user" << endl;
		ros::shutdown();
	}
}

void imageThresholdCallback(const sensor_msgs::CompressedImage& msg)
{
	Mat img_stream;
	img_stream = cv::imdecode(cv::Mat(msg.data),1);
	if (!img_stream.empty()){
		cv::imshow("Compressed Video Threshold", img_stream);
	}
		
	if (cv::waitKey(30) == 27){
		cout << "esc key is pressed by user" << endl;
		ros::shutdown();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "system_monitor");
  ros::NodeHandle nh;
  namedWindow("Compressed Video Original", cv::WINDOW_NORMAL);
  namedWindow("Compressed Video Threshold", cv::WINDOW_NORMAL);
  startWindowThread();
  enableControl();
  ros::Subscriber sub_original = nh.subscribe("auvsi16/testing/video_original/compressed", 1, imageOriginalCallback,ros::TransportHints().unreliable()); // setting queue to 1 remove delay in compressedImage data | also using UDP
  ros::Subscriber sub_threshold = nh.subscribe("auvsi16/testing/video_threshold/compressed", 1, imageThresholdCallback,ros::TransportHints().unreliable()); // setting queue to 1 remove delay in compressedImage data | also using UDP
  pub_control = nh.advertise<std_msgs::Int32MultiArray>("auvsi16/testing/control_data", 1);

  ros::spin();
  destroyWindow("Compressed Video Original");
  destroyWindow("Compressed Video Threshold");
  }
