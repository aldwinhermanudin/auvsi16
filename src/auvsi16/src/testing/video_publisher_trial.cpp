/************************************************************************************
 * Nama Program	:	video_publisher													*
 * Last editor	:	Arief Purnama Muharram											*
 * Last edit	:	21/01/2016														*
 * Referensi	:	http://wiki.ros.org/image_transport/Tutorials/PublishingImages	*
 * **********************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32MultiArray.h>

int iLowH 	= 15;	// yellow = 15, red = 166
int iHighH 	= 29;	// yellow = 29, red = 179
	
int iLowS	= 90; // lower this to allow more noise, 80 is normal
int iHighS 	= 184;
	
int iLowV 	= 138; // lower this to allow more noise
int iHighV 	= 255;

using namespace std;
using namespace cv;

void controldataCallBack(const std_msgs::Int32MultiArray& control_data);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_feed"); // node name: video_publisher
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_video_threshold = it.advertise("auvsi16/testing/video_threshold", 1); // topic: auvsi16/video; Queue: 1
  image_transport::Publisher pub_video_original = it.advertise("auvsi16/testing/video_original", 1); // topic: auvsi16/video; Queue: 1
  
  ros::Subscriber sub_control_data = nh.subscribe("auvsi16/testing/control_data", 1, controldataCallBack,ros::TransportHints().unreliable());
  
  cv::VideoCapture cap(1); 					// set video source to video0
  // Camera Check; Error handling
  if(!cap.isOpened()) {
	  ROS_ERROR ("Error opening camera.");	  
	  return 1;
  }
  
  cv::Mat frame;
  sensor_msgs::ImagePtr image_msg_original;
  sensor_msgs::ImagePtr image_msg_threshold;
  
  while (nh.ok()) {
    cap.read(frame);
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
		
    image_msg_original = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	GaussianBlur( frame, frame , Size( 5, 5 ), 0, 0 );
	
	Mat imgHSV;
	cvtColor(frame, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	
	Mat imgThresholded;
	
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	
	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
	//morphological closing (removes small holes from the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
    image_msg_threshold = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgThresholded).toImageMsg();
	pub_video_threshold.publish(image_msg_threshold);
    pub_video_original.publish(image_msg_original);
    }

    ros::spinOnce();
  }
}

void controldataCallBack(const std_msgs::Int32MultiArray& control_data){
	iLowH 	= control_data.data[0];	// yellow = 15, red = 166
	iHighH 	= control_data.data[1];	// yellow = 29, red = 179
	
	iLowS	= control_data.data[2];// lower this to allow more noise, 80 is normal
	iHighS 	= control_data.data[3];
		
	iLowV 	= control_data.data[4]; // lower this to allow more noise
	iHighV 	= control_data.data[5];	
	
}
