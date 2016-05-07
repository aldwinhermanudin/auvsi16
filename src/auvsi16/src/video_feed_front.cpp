#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>

void v4l2Configuration(const char* camera_port,int exposure_mode,int exposure_value, 
						int white_balance_mode, int white_balance_value, 
						int brightness, int contrast,
						int saturation, int sharpness, int gain);
						
void getCameraParameter(ros::NodeHandle nh_main);
void setCameraConfiguration();
	
int camera_port = 0;
int camera_width = 1280;
int camera_height = 720;
	
int v4l2_configuration = 0;
int v4l2_exposure_mode = 0;
int v4l2_exposure_value = 0;
int v4l2_white_balance_mode = 1;
int v4l2_white_balance_value = 0;
int v4l2_brightness = 128;
int v4l2_contrast = 128;
int v4l2_saturation = 128; 
int v4l2_sharpness = 128; 
int v4l2_gain = 128;
	
double resize_width_factor = 0.5;
double resize_height_factor = 0.5;

using namespace cv;

int main(int argc, char** argv){

	ros::init(argc, argv, "video_feed_front"); // node name: video_publisher
	ros::NodeHandle nh("~");
	
 	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub_video = it.advertise("/auvsi16/video/front", 1, true); 	// topic: auvsi16/video; Queue: 1; Latch :true
	
	getCameraParameter(nh);
	setCameraConfiguration();
	cv::VideoCapture cap(camera_port); 	// set video source to video0
	cap.set(CV_CAP_PROP_FRAME_WIDTH, camera_width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera_height);
	
	// Camera Check; Error handling //
	if(!cap.isOpened()) {
		ROS_ERROR ("Error opening camera.");	  
		return 1;
	}
	  
	cv::Mat frame;
	sensor_msgs::ImagePtr image_msg;

	while (nh.ok()) {
		cap.read(frame);
		if(!frame.empty()){
			image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			pub_video.publish(image_msg);
		}
		ros::spinOnce();
	}
}

void setCameraConfiguration(){
	
	if (v4l2_configuration == 0){
		ROS_INFO("[VFF] V4L2 Configuration Disabled");
	}
	else if (v4l2_configuration == 1){
		ROS_INFO("[VFF] V4L2 Configuration Enabled\n");
		char video_port[] = "/dev/video0";
		video_port[10] = camera_port+48;
		
		if (v4l2_exposure_mode == 0) v4l2_exposure_mode = V4L2_EXPOSURE_APERTURE_PRIORITY;
		else if (v4l2_exposure_mode == 1) v4l2_exposure_mode = V4L2_EXPOSURE_MANUAL;
		
		v4l2Configuration(video_port,v4l2_exposure_mode, v4l2_exposure_value,v4l2_white_balance_mode,v4l2_white_balance_value,v4l2_brightness,v4l2_contrast,v4l2_saturation,v4l2_sharpness,v4l2_gain);
		
	}	
	
}

void getCameraParameter(ros::NodeHandle nh_main){	
	nh_main.getParam("camera_port", camera_port);
	nh_main.getParam("camera_width", camera_width);
	nh_main.getParam("camera_height", camera_height);
		
	nh_main.getParam("v4l2_configuration", v4l2_configuration);
	nh_main.getParam("v4l2_exposure_mode", v4l2_exposure_mode);
	nh_main.getParam("v4l2_exposure_value", v4l2_exposure_value);
	nh_main.getParam("v4l2_white_balance_mode", v4l2_white_balance_mode);
	nh_main.getParam("v4l2_white_balance_value", v4l2_white_balance_value);
	nh_main.getParam("v4l2_brightness", v4l2_brightness);
	nh_main.getParam("v4l2_contrast", v4l2_contrast);
	nh_main.getParam("v4l2_saturation", v4l2_saturation);
	nh_main.getParam("v4l2_sharpness", v4l2_sharpness);
	nh_main.getParam("v4l2_gain", v4l2_gain);
	
}

void v4l2Configuration(const char* camera_port,int exposure_mode,int exposure_value, 
						int white_balance_mode, int white_balance_value, 
						int brightness, int contrast,
						int saturation, int sharpness, int gain){
	
	int descriptor;
    if((descriptor = v4l2_open(camera_port, O_RDWR)) < 0){
        ROS_ERROR("Error Opening Camera");
    }
    v4l2_control c;
    
   // Exposure control setting | V4L2_EXPOSURE_MANUAL or V4L2_EXPOSURE_APERTURE_PRIORITY | Default : AUTO
	c.id = V4L2_CID_EXPOSURE_AUTO;
	c.value = exposure_mode;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0){
		if(exposure_mode == V4L2_EXPOSURE_MANUAL)
			ROS_INFO("EXPOSURE_AUTO success. Value : EXPOSURE_MANUAL");
			
		else if(exposure_mode == V4L2_EXPOSURE_APERTURE_PRIORITY)
			ROS_INFO("EXPOSURE_AUTO success. Value : APERTURE_PRIORITY");
	}
	
   	// Exposure control | 3 - 2047 | Default : off
	c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	c.value = exposure_value;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
		ROS_INFO("EXPOSURE_ABSOLUTE success. Value : %d",exposure_value);
		
	// Auto white balance control | 1 or 0 | Default : 1
	c.id = V4L2_CID_AUTO_WHITE_BALANCE ;
	c.value = white_balance_mode;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
		ROS_INFO("AUTO_WHITE_BALANCE success. Value : %d",white_balance_mode);

	// White balance value control | 2000 - 6500 | Default : off
	c.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE ;
	c.value = white_balance_value;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
		ROS_INFO("WHITE_BALANCE_TEMPERATURE success. Value : %d",white_balance_value);
		
	// Brightness control | 0 - 255 | Default : 128
	c.id = V4L2_CID_BRIGHTNESS ;
	c.value = brightness;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
		ROS_INFO("BRIGHTNESS success. Value : %d",brightness);
    
	// Contrast control | 0 - 255 | Default : 128
	c.id = V4L2_CID_CONTRAST ;
	c.value = contrast;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
		ROS_INFO("CONTRAST success. Value : %d",contrast);
    
	// Saturation control | 0 - 255 | Default : 128
	c.id = V4L2_CID_SATURATION ;
	c.value = saturation;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
		ROS_INFO("SATURATION success. Value : %d",saturation);

	// Sharpness control | 0 - 255 | Default : 128
	c.id = V4L2_CID_SHARPNESS ;
	c.value = sharpness;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
		ROS_INFO("SHARPNESS success. Value : %d",sharpness);
	
	// Gain value control | 0 - 255 | Default : 128
	c.id = V4L2_CID_GAIN ;
	c.value = gain;
	if(v4l2_ioctl(descriptor, VIDIOC_S_CTRL, &c) == 0)
		ROS_INFO("GAIN success. Value : %d",gain);
		
    close(descriptor);
}
