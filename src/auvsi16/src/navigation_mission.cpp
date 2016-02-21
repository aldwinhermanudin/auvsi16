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
	pid_in.setpoint = 640;
	
	namedWindow("Control", CV_WINDOW_NORMAL); //create a window called "Control"
	namedWindow("Thresholded Image", CV_WINDOW_NORMAL); //create a window called "Control"
	namedWindow("Original", CV_WINDOW_NORMAL); //create a window called "Control"
	
	int iLowH = 58;
	int iHighH = 70;

	int iLowS = 101; 
	int iHighS = 179;

	int iLowV = 99;
	int iHighV = 255;
	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);

	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);
	sleep(5);
	
	while (ros::ok()){
		
		ros::spinOnce();	// read frame

		Mat imgHSV;

		cvtColor(image_received, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		 
		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
			  
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000){
				
			//calculate the position of the ball
			int posX = dM10 / dArea;
			
			pid_in.x = posX;
			pid_in.t = pid_in.t+delta_t;
			
			pub_pid_in.publish(pid_in);
			
			ros::spinOnce();
			
			motor_control.steering = 1500 + (0 - pid_out);
			motor_control.throttle = 1600;
			pub_ovrd_mtr.publish(motor_control);
			   
			cout << "######## Posisi X dan Y #########" << endl;
			cout << "Posisi X : " << posX << endl;
			cout << endl;
			
		}
		
		
	   imshow("Thresholded Image", imgThresholded); //show the thresholded image
	   imshow("Original", image_received); //show the original image

		if (waitKey(30) == 27){
			cout << "esc key is pressed by user" << endl;
			break; 
		}
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
