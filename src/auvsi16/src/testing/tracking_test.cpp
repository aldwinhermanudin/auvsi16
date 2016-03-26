#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <pid/plant_msg.h>
#include <pid/pid_const_msg.h>
#include <pid/controller_msg.h>
#include <auvsi16/overrideMotorRC.h>
#include <auvsi16/sonarData.h>

using namespace std;
using namespace cv;

cv::Mat image_received;

void imageReceiveCB(const sensor_msgs::ImageConstPtr& msg);
void pidOutCB(const pid::controller_msg& msg);
void sonarDataCB(const auvsi16::sonarData& msg);
int imageProcessing(Mat imgInput, int low_hue, int high_hue, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius);

double pid_out;
int sonar_data[13];
auvsi16::overrideMotorRC motor_control;
pid::plant_msg pid_in;
pid::pid_const_msg pid_const;
float Kp = 0.9;
float Ki = 0.1 ;
float Kd = 0.02 ;
double t_IC = 0.0;
double delta_t = 0.01;

int center_buoy_x	= 0;
int center_buoy_y 	= 0;
double buoy_area	= 0;
double radius_buoy	= 0;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "navigation_mission");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	ros::Publisher	pub_pid_in = nh.advertise<pid::plant_msg>("auvsi16/pid/in", 1);
	ros::Publisher 	pub_pid_const = nh.advertise<pid::pid_const_msg>("auvsi16/pid/constant", 1,true);
	ros::Publisher 	pub_ovrd_mtr = nh.advertise<auvsi16::overrideMotorRC>("auvsi16/overrideMotorRC", 1);
	
	ros::Subscriber sub_pid_out = nh.subscribe("auvsi16/pid/out", 1, pidOutCB);
	ros::Subscriber sub_sonar_data = nh.subscribe("auvsi16/sonar_data", 1, sonarDataCB);
	image_transport::Subscriber sub = it.subscribe("auvsi16/video", 1, imageReceiveCB); // Topik subscribe: auvsi16/video
	
	pid_const.p = Kp;
	pid_const.i = Ki;
	pid_const.d = Kd;
	pub_pid_const.publish(pid_const);
	pid_in.setpoint = 320;
	
	sleep(5);
	
	while (ros::ok()){
		
		ros::spinOnce();	// read frame
		
		int buoy_number = imageProcessing(image_received, 15, 29,0, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy);
		
		RNG rng(12345);
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		Point2f circle_center;
		circle_center.x = center_buoy_x;
		circle_center.y = center_buoy_y;
		circle( image_received, circle_center, radius_buoy, color, 4, 8, 0 );
		waitKey(30);
		imshow("Image", image_received );
		
		cout << "######## Posisi Yellow #########" << endl;
		cout << "Detected Buoy : " << buoy_number << endl;
		cout << "Posisi X  : " << center_buoy_x << endl;
		cout << "Posisi Y  : " << center_buoy_y << endl;
		cout << "Bouy Size : " << buoy_area << endl;
		cout << endl;
		cout << endl;
		
		pid_in.x = center_buoy_x;
		pid_in.t = pid_in.t+delta_t;
		
		pub_pid_in.publish(pid_in);
		
		ros::spinOnce();
		
		motor_control.steering = 1500 + (0 - pid_out); // negative for left turn, positif for right turn
		motor_control.throttle = 1500;
		pub_ovrd_mtr.publish(motor_control);
	}
}

void imageReceiveCB(const sensor_msgs::ImageConstPtr& msg){
	
	try{
		image_received  = cv_bridge::toCvCopy(msg, "bgr8")->image;		// use cv_bridge::toCvCopy instead of cv_bridge::toCvShare to process a mat data
	}
	
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void pidOutCB(const pid::controller_msg& pid_out_recv){
	
	pid_out = pid_out_recv.u;
	
}

void sonarDataCB(const auvsi16::sonarData& sonar_recv){
	
	for(int i=0; i < 13; i++) sonar_data[i] = sonar_recv.data[i];
}



int imageProcessing(Mat imgInput, int low_hue, int high_hue, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius){
	
	Mat imgOriginal = imgInput.clone();
	int iLowH 	= low_hue;	// yellow = 15, red = 166
	int iHighH 	= high_hue;	// yellow = 29, red = 179
	
	int iLowS	= 90; // lower this to allow more noise, 80 is normal
	int iHighS 	= 184;
	
	int iLowV 	= 138; // lower this to allow more noise
	int iHighV 	= 255;
	
	GaussianBlur( imgOriginal, imgOriginal, Size( 5, 5 ), 0, 0 );
	
	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	
	Mat imgThresholded;
	
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	
	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
	//morphological closing (removes small holes from the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Point2f center;
	float radius;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	
	int largest_bouy_area = 0;
	int largest_bouy_id = 0;
	int number_of_detected_buoy = 0;
	for( int i = 0; i< contours.size(); i++ ){
		double a = contourArea( contours[i],false);  //  Find the area of contour
		if(a > largest_bouy_area && a > minimum_area){
			largest_bouy_area = a;
			largest_bouy_id = i;                //Store the index of largest contour
			minEnclosingCircle( contours[i], center, radius);
		}
		if(largest_bouy_area > minimum_area){
			number_of_detected_buoy++;
		}
		
	}
	if(largest_bouy_area > minimum_area){
		
		*detected_center_x	=	(int)center.x;
		*detected_center_y	=	(int)center.y;
		*contour_area		=	largest_bouy_area;
		*detected_radius	=	radius;
	}
	
	return number_of_detected_buoy;
}
