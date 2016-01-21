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
  ros::init(argc, argv, "video_publisher"); // Nama program: video_publisher
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("AUVSI16/media/video", 10000); // Topik publish: AUVSI16//media/video; Queue: 1000


  int video0;
  cv::VideoCapture cap(video0); // Ambil video dari file video0
  
  // Cek kamera; Error handling
  if(!cap.isOpened()) {
	  ROS_ERROR ("Camera Error");	  
	  return 1;
  }
  
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(100);
  while (nh.ok()) {
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
