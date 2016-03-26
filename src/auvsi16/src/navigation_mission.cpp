#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/NavSatFix.h>
#include <pid/plant_msg.h>
#include <pid/pid_const_msg.h>
#include <pid/controller_msg.h>
#include <auvsi16/overrideMotorRC.h>
#include <auvsi16/sonarData.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/WaypointSetCurrent.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/VFR_HUD.h"
#include "auvsi16/waypointSet.h"
#include "geometry_msgs/TwistStamped.h"

#define SPIN_BASESPEED 1495

using namespace std;
using namespace cv;
using namespace GeographicLib;

cv::Mat image_received;

void imageReceiveCB(const sensor_msgs::ImageConstPtr& msg);
void gpsVelocityCB(const geometry_msgs::TwistStamped& msg);
void pidOutCB(const pid::controller_msg& msg);
void compassCB(const mavros_msgs::VFR_HUD& msg);
void sonarDataCB(const auvsi16::sonarData& msg);
void globalPositionCB(const sensor_msgs::NavSatFix& msg);
int imageProcessing(Mat imgInput, int low_hue, int high_hue, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius);

bool changeFlightMode(const char* flight_mode);
void calculateCoordinate(const double lat0, const double lon0, const  double h0,
						 double *lat_target,double *lon_target,
						 double x_target, double y_target);


void positionEstimation(float distance, float angle, long double *x_final, long double *y_final);

bool areaCheck(int input_x, int setpoint, int area_limit);
bool moveForward(int shift_x);
void motorControl(int setpoint, int input_x, int base_speed, int steer_correction);
double calculateCenterLine(int first_x, int second_x);
void changePID(float Kp_input, float Ki_input, float Kd_input);
void headingControl(int heading, int setpoint_heading);
void overrideRCControl(int setpoint, int input_x, int base_speed, int steer_correction);

int first_buoy_x			= 0;
int first_buoy_y 			= 0;
double first_buoy_area		= 0;
double first_radius_buoy	= 0;
int second_buoy_x			= 0;
int second_buoy_y 			= 0;
double second_buoy_area		= 0;
double second_buoy_radius	= 0;

double gps_vel_x;
double gps_vel_y;
double gps_vel_z;
double compass_hdg;
double pid_out;
int sonar_data[13];
sensor_msgs::NavSatFix global_position;

ros::Publisher	pub_pid_in ;
ros::Publisher 	pub_pid_const ;
ros::Publisher 	pub_ovrd_mtr;
ros::ServiceClient client_wp_set;
ros::ServiceClient client_set_flightmode;
double t_IC = 0.0;

// Testing
void inputHeadingCB(const std_msgs::Int32& msg);
int input_heading = 0;
// Testing

int main(int argc, char **argv){
	
	ros::init(argc, argv, "navigation_mission");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	pub_pid_in = nh.advertise<pid::plant_msg>("auvsi16/pid/in", 1);
	pub_pid_const = nh.advertise<pid::pid_const_msg>("auvsi16/pid/constant", 1,true);
	pub_ovrd_mtr = nh.advertise<auvsi16::overrideMotorRC>("auvsi16/overrideMotorRC", 1);
	
	client_wp_set = nh.serviceClient<auvsi16::waypointSet>("/auvsi16/waypoint_set_server");
	client_set_flightmode = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	
	ros::Subscriber sub_gps_vel = nh.subscribe("/mavros/global_position/raw/gps_vel", 1, gpsVelocityCB);
	ros::Subscriber sub_pid_out = nh.subscribe("auvsi16/pid/out", 1, pidOutCB);
	ros::Subscriber sub_compass = nh.subscribe("/mavros/vfr_hud", 1, compassCB);
	ros::Subscriber sub_global_position = nh.subscribe("/mavros/global_position/global", 1, globalPositionCB);
	ros::Subscriber sub_sonar_data = nh.subscribe("auvsi16/sonar_data", 1, sonarDataCB);
	image_transport::Subscriber sub = it.subscribe("auvsi16/video", 1, imageReceiveCB); // Topik subscribe: auvsi16/video
	
	usleep(100000);
    changePID(0.3, 0, 0);
	
	// Testing
	ros::Subscriber sub_input_heading = nh.subscribe("auvsi16/testing/input_heading", 1, inputHeadingCB);
	//changePID(3.5, 0, 0);
	// Testing

	
	while (ros::ok()){
		
		// Testing
		//ros::spinOnce();
		//changePID(3.5, 0, 0);
		//headingControl(compass_hdg, input_heading);		
		//usleep(500000); // slow things down
		// Testing
		
		/*
		ros::spinOnce();	// read frame
		int green_buoy = imageProcessing(image_received, 15, 29,0, &first_buoy_x, &first_buoy_y, &first_buoy_area, &first_radius_buoy);
		int red_buoy = imageProcessing(image_received, 166, 179,0, &second_buoy_x, &second_buoy_y, &second_buoy_area, &second_buoy_radius);
	
		ROS_WARN_STREAM("Detected Green Buoy : " << green_buoy);
		ROS_WARN_STREAM("Detected Red Buoy : " << red_buoy);
		double center_line = calculateCenterLine(first_buoy_x,second_buoy_x);
		if (areaCheck(center_line, 320, 10)){
			moveForward(10);
			changeFlightMode("AUTO");
			// if flightmode is HOLD, continue code
		}
		overrideRCControl(320, center_line, 1500, 0);
		*/
		
		// hue saturation visibility parameter for imageProcessing();
		
		ros::spinOnce();	// read frame
		int red_buoy = imageProcessing(image_received, 166, 179,0, &second_buoy_x, &second_buoy_y, &second_buoy_area, &second_buoy_radius);
		ROS_INFO_STREAM("Compass Heading : " << compass_hdg);
		ROS_INFO_STREAM("Latitude : " << global_position.latitude);
		ROS_INFO_STREAM("Longitude : " <<  global_position.longitude);
		ROS_INFO_STREAM("X Position : " << second_buoy_x);
		//ROS_WARN_STREAM("Detected Red Buoy : " << red_buoy);
		if (areaCheck(second_buoy_x, 320, 2)){
			if(moveForward(20)){
				ROS_WARN_STREAM("Set Waypoint Success!");
			}
			else {
				ROS_WARN_STREAM("Set Waypoint Failed!");
			}
			
			ROS_WARN_STREAM("Destination Set!");
			ros::shutdown();
			// changeFlightMode("AUTO");
			// if flightmode is HOLD, continue code
		}
		overrideRCControl(320, second_buoy_x, SPIN_BASESPEED, 0);
		
	}
}

void imageReceiveCB(const sensor_msgs::ImageConstPtr& msg){
	
	try {
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

void compassCB(const mavros_msgs::VFR_HUD& msg){
	
	compass_hdg = msg.heading;
	
}

// Testing
void inputHeadingCB(const std_msgs::Int32& msg){
	
	input_heading = msg.data;
	
}
// Testing

void gpsVelocityCB(const geometry_msgs::TwistStamped& msg){
	
	gps_vel_x = msg.twist.linear.x;
	gps_vel_y = msg.twist.linear.y;
	gps_vel_z = msg.twist.linear.z;
	
}

void globalPositionCB(const sensor_msgs::NavSatFix& msg){
	
	global_position.latitude = msg.latitude;
	global_position.longitude = msg.longitude;
	global_position.altitude = msg.altitude;
	
}

int imageProcessing(Mat imgInput, int low_hue, int high_hue, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius){
	
	Mat imgOriginal = imgInput.clone();
	int iLowH 	= low_hue;	// yellow = 15, red = 166
	int iHighH 	= high_hue;	// yellow = 29, red = 179
	
	int iLowS	= 40; // lower this to allow more noise, 80 is normal
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
void changePID(float Kp_input, float Ki_input, float Kd_input){
	pid::pid_const_msg pid_const;

	if(Kp_input != pid_const.p || Ki_input != pid_const.i || Kd_input != pid_const.d){
		pid_const.p = Kp_input;
		pid_const.i = Ki_input;
		pid_const.d = Kd_input;
		pub_pid_const.publish(pid_const);
	}
}

void motorControl(int setpoint, int input_x, int base_speed, int steer_correction){
	pid::plant_msg pid_in;
	auvsi16::overrideMotorRC motor_control;
	double delta_t = 0.01;
	pid_in.setpoint = setpoint;
	pid_in.x = input_x;
	pid_in.t = pid_in.t+delta_t;
		
	pub_pid_in.publish(pid_in);
		
	ros::spinOnce();
		
	motor_control.steering = 1526 + (steer_correction + pid_out); // negative for left turn, positif for right turn
	motor_control.throttle = base_speed;
	pub_ovrd_mtr.publish(motor_control);
}


double calculateCenterLine(int first_x, int second_x){
	
	int left_x, right_x;
	if(first_x > second_x){
		left_x = second_x;
		right_x = first_x;
	}
	
	else {
		left_x = first_x;
		right_x = second_x;
	}
	
	double center_line = abs(left_x - right_x)/2 + left_x;
	
	return center_line;
}

bool areaCheck(int input_x, int setpoint, int area_limit){
	
	int lower_limit = setpoint - area_limit;
	int upper_limit = setpoint + area_limit;
	
	if(input_x <= upper_limit && input_x >= lower_limit){
		return true;
	}
	
	else {
		return false;
	}
}

void positionEstimation(float distance, float angle, long double *x_final, long double *y_final){
   *x_final = (distance * sinl( angle * M_PIl / 180.0 )); //persamaan aslinya X = r . cos(theta-90)
   *y_final = (distance * cosl( angle * M_PIl / 180.0 )); //persamaan aslinya Y = r . sin(theta+90)
}

void calculateCoordinate(const double lat0, const double lon0, const double h0,
						 double *lat_target,double *lon_target,
						 double x_target, double y_target){
  //const double lat0 = -6.36248605092425, lon0 = 106.82503312826157; // current location of boat
  
  Utility::set_digits();
  Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
  LocalCartesian proj(lat0, lon0, h0, earth);
  double h_target; // target location
      
  // Sample reverse calculation
  proj.Reverse(x_target, y_target, 0, *lat_target, *lon_target, h_target);
}

bool moveForward(int shift_x){
	auvsi16::waypointSet wp_set;
	long double x_target;
	long double y_target;
	double target_latitude;
	double target_longitude;
	
	positionEstimation(shift_x,compass_hdg,&x_target, &y_target);
	calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);
	wp_set.request.x_lat = target_latitude;
	wp_set.request.y_long = target_longitude;
	bool success_set = client_wp_set.call(wp_set);

	// Check for success and use the response .
	if(success_set){
		return true;
	} 
	else {
		return false;
	}
}

bool changeFlightMode(const char* flight_mode){
	mavros_msgs::SetMode flightmode;
	flightmode.request.base_mode = 0;				//Set to 0 to use custom_mode
	flightmode.request.custom_mode = flight_mode;		//Set to '' to use base_mode
	bool success = client_set_flightmode.call(flightmode);

	// Check for success and print out info.
	if(success){
		return true;
		//ROS_INFO_STREAM( "Flight Mode changed to "<< flight.request.custom_mode ) ;
	} 
	else {
		return  false;
		//ROS_ERROR_STREAM( "Failed to changed." ) ;
	}
}

void headingControl(int heading, int setpoint_heading){

	double zero_degree_shift	= heading - setpoint_heading;
	if (zero_degree_shift < 0){
		zero_degree_shift = zero_degree_shift + 360;
	}
	double delta_zero_degree	= zero_degree_shift;
	double delta_360_degree		= 360-zero_degree_shift;
	if (delta_zero_degree > delta_360_degree){
		heading = 0 - delta_360_degree; // turn left
	}
	else if (delta_360_degree > delta_zero_degree){
		heading = delta_zero_degree; //turn right
	}
	
	motorControl(0, heading, SPIN_BASESPEED, 0); // pid controller, (Setpoint - X)
		
}

void overrideRCControl(int setpoint, int input_x, int base_speed, int steer_correction){
	int error;
	if(input_x > setpoint){
		error = input_x - setpoint;
		input_x = setpoint - error;
	}
	else if(input_x < setpoint){
		error = setpoint - input_x;
		input_x = setpoint + error;
	}
	
	motorControl(setpoint, input_x, base_speed, steer_correction);		
}
