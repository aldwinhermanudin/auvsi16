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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_feed"); // node name: video_publisher
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_video = it.advertise("auvsi16/video", 1); // topic: auvsi16/video; Queue: 1

  cv::VideoCapture cap(0); 					// set video source to video0
  // Camera Check; Error handling
  if(!cap.isOpened()) {
	  ROS_ERROR ("Error opening camera.");	  
	  return 1;
  }
  
  cv::Mat frame;
  sensor_msgs::ImagePtr image_msg;
  
  while (nh.ok()) {
    cap.read(frame);
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub_video.publish(image_msg);
    }

    ros::spinOnce();
  }
}
