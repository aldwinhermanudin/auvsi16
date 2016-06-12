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
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/VFR_HUD.h"
#include "geometry_msgs/TwistStamped.h"
#include <boost/shared_ptr.hpp>
#include "waypoint_sender.hpp"

#define BASESPEED 	1495
#define YES 		1
#define NO			0

using namespace std;
using namespace cv;
using namespace GeographicLib;

void setBMFConfiguration		(ros::NodeHandle nh_main);
void pidOutCB								(const pid::controller_msg& msg);
void pidConstantCB					(const pid::pid_const_msg& msg);
void compassCB							(const std_msgs::Float64& msg);
void globalPositionCB				(const sensor_msgs::NavSatFix& msg);

bool changeFlightMode				(const char* flight_mode);
void calculateCoordinate		(const double lat0, const double lon0, const  double h0, double *lat_target,double *lon_target,double x_target, double y_target);

void positionEstimation			(float distance, float angle, long double *x_final, long double *y_final);
bool areaCheck							(int input_x, int setpoint, int area_limit);
bool moveForward						(int shift_x);
void motorControl						(int setpoint, int input_x, int base_speed, int steer_correction);
double calculateCenterLine	(int first_x, int second_x);
void changePID							(float Kp_input, float Ki_input, float Kd_input);
void headingControl					(int heading, int setpoint_heading);
void overrideRCControl			(int setpoint, int input_x, int base_speed, int steer_correction);
void addWaypoint						(double latitude, double longitude);
bool moveToHeading					(int shift_x, int heading);

double 						pid_out;
double 						compass_hdg;
sensor_msgs::NavSatFix		global_position;
pid::pid_const_msg			pid_const;
cv::Mat 					image_received;

ros::Publisher			pub_pid_in ;
ros::Publisher 			pub_pid_const ;
ros::Publisher 			pub_ovrd_mtr;
ros::ServiceClient 	client_set_flightmode;
ros::Subscriber 		sub_pid_out;
ros::Subscriber 		sub_pid_constant;
ros::Subscriber 		sub_compass;
ros::Subscriber 		sub_global_position;
boost::shared_ptr<WaypointSender> wp_sender;

double t_IC = 0.0;

void setBMFConfiguration(ros::NodeHandle nh_main){

	pub_pid_in 		= nh_main.advertise<pid::plant_msg>("/auvsi16/pid/in", 1);
	pub_pid_const 	= nh_main.advertise<pid::pid_const_msg>("/auvsi16/pid/constant", 1,true);
	pub_ovrd_mtr	= nh_main.advertise<auvsi16::overrideMotorRC>("/auvsi16/overrideMotorRC", 1);

	client_set_flightmode 	= nh_main.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	// move subscriber declaration to global
	sub_pid_out				 = nh_main.subscribe("/auvsi16/pid/out", 1, pidOutCB);
	sub_pid_constant		 = nh_main.subscribe("/auvsi16/pid/constant", 1, pidConstantCB);
	sub_compass				 = nh_main.subscribe("/mavros/global_position/compass_hdg", 1, compassCB);
	sub_global_position		 = nh_main.subscribe("/mavros/global_position/global", 1, globalPositionCB);

	wp_sender.reset(new WaypointSender());
}

void pidConstantCB(const pid::pid_const_msg& msg){

	pid_const.p = msg.p;
	pid_const.i = msg.i;
	pid_const.d = msg.d;
}

void pidOutCB(const pid::controller_msg& pid_out_recv){

	pid_out = pid_out_recv.u;
}

void compassCB(const std_msgs::Float64& msg){

	compass_hdg = msg.data;
}

void globalPositionCB(const sensor_msgs::NavSatFix& msg){

	global_position.latitude = msg.latitude;
	global_position.longitude = msg.longitude;
	global_position.altitude = msg.altitude;
}

void changePID(float Kp_input, float Ki_input, float Kd_input){

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
   *x_final = (distance * sinl( angle * (M_PIl / 180.0) )); //persamaan aslinya X = r . cos(theta-90)
   *y_final = (distance * cosl( angle * (M_PIl / 180.0) )); //persamaan aslinya Y = r . sin(theta+90)
}

void calculateCoordinate(const double lat0, const double lon0, const double h0, double *lat_target,double *lon_target, double x_target, double y_target){
  Utility::set_digits();
  Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
  LocalCartesian proj(lat0, lon0, h0, earth);
  double h_target; // target location

  // Sample reverse calculation
  proj.Reverse(x_target, y_target, 0, *lat_target, *lon_target, h_target);
}

bool moveForward(int shift_x){
	long double x_target;
	long double y_target;
	double target_latitude;
	double target_longitude;

	positionEstimation(shift_x,compass_hdg,&x_target, &y_target);
	calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);

	wp_sender->clearWaypointList();	// clear waypoint list
	wp_sender->addWaypoint(target_latitude, target_longitude);	// add waypoint to list
	bool success_set = wp_sender->sendWaypointList();	// send waypoint to fcu

	// Check for success and use the response .
	if(success_set){
		return true;
	}
	else {
		return false;
	}
}

bool moveToHeading(int shift_x, int heading){
	long double x_target;
	long double y_target;
	double target_latitude;
	double target_longitude;

	positionEstimation(shift_x,heading,&x_target, &y_target);
	calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);

	wp_sender->addWaypoint(target_latitude, target_longitude);	// add waypoint to list
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

	motorControl(0, heading, BASESPEED, 0); // pid controller, (Setpoint - X)
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
