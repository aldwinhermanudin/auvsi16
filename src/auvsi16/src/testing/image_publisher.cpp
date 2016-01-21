/********************************************
 * Nama Program	:	image_publisher			*
 * Last editor	:	Arief Purnama Muharram	*
 * Last edit	:	21/01/2016				*
 * Referensi	:	Unknown					*
 * ******************************************/
 
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher"); // Nama program: image_publisher
	ros::NodeHandle nh;

	cv_bridge::CvImage cv_image;
	cv_image.image = cv::imread("/home/comlab1/Documents/Arief-Purnama-Muharram/ROS/examples/src/example_opencv/src/pictures_example/FKUI.jpg",CV_LOAD_IMAGE_COLOR); // Alamat gambar
	cv_image.encoding = "bgr8";
	sensor_msgs::Image ros_image;
	cv_image.toImageMsg(ros_image);

	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("AUVSI16/media/image", 1); // Topik publish: AUVSI16//media/image; Queue: 1
	ros::Rate loop_rate(5);

	while (nh.ok()) 
	{
		pub.publish(ros_image);
		loop_rate.sleep();
	}
}
