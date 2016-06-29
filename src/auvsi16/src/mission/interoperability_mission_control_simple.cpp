#include "../../include/auvsi16/basic_mission_function.hpp"
#include "../../include/auvsi16/auvsi16_general.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <auvsi16/ImageProcessing.h>
#include <auvsi16/RobosubLauncher.h>
#include "mavros_msgs/State.h"

void nodeSelectCB(const std_msgs::String& msg);
void imageProcessingCB				(const auvsi16::ImageProcessing& msg);
void stateCB									(const mavros_msgs::State& msg);
bool checkAUTOCruise					();

ros::Publisher pub_imgproc_select;

bool		imgproc_status = false;
int 		center_buoy_x	= 0;
int 		center_buoy_y = 0;
double	buoy_area			= 0;
double 	radius_buoy		= 0;
int 		buoy_number  	= 0;

std_msgs::String node_status;
std_msgs::String node_feedback;
string										state;
int main(int argc, char **argv){

	ros::init(argc, argv, "interoperability_mission_simple");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	setBMFConfiguration(nh);

	ImageProcessingInterface imgproc_interface;
	auvsi16::RobosubLauncher	robosub;
	
	changePID(0.3, 0, 0);

	ros::Subscriber 						sub_state 			= nh.subscribe("/mavros/state", 1, stateCB);
	

	ros::Publisher pub_robosub		= nh.advertise<auvsi16::RobosubLauncher>("/auvsi16/robosub/launcher/control", 16);
	ros::Publisher pub_run_status		= nh.advertise<std_msgs::String>("/auvsi16/mission/navigation/status", 16);
	ros::Publisher pub_node_select 	= nh.advertise<std_msgs::String>("/auvsi16/node/select", 16,true);
	ros::Subscriber sub_node_select = nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);

	ros::Subscriber sub_imgproc_data	= nh.subscribe("/auvsi16/node/image_processing/data", 10, imageProcessingCB);

	ROS_WARN_STREAM("Waiting for interoperability mission selected.");
	while (ros::ok() && node_status.data != "im:interoperability.start"){
		ros::spinOnce();
	}

	ROS_WARN_STREAM("Interoperability mission selected.");

	imgproc_interface.configuration(3);
	ROS_INFO("Waiting for Image Processing Node!");
	// this it to whether image_received is empty or not, move this to a function.
	// while (ros::ok() && front_image.empty() && right_image.empty()){
	while (ros::ok() && !imgproc_status){
		ros::spinOnce();
	}
		imgproc_interface.configuration(0);

	// tells robosub to go full speed here
	
	ROS_WARN_STREAM("Rolling out robosub.");
	robosub.motor_direction = 1;
	robosub.motor_speed = 70;
	pub_robosub.publish(robosub);
	sleep(12);
	
	// wait until receive something here
	
	ROS_WARN_STREAM("Rolling in robosub.");
	robosub.motor_direction = 2;
	robosub.motor_speed = 70;
	pub_robosub.publish(robosub);
	sleep(12);
	robosub.motor_direction = 0;
	robosub.motor_speed = 125;
	pub_robosub.publish(robosub);
	
	node_feedback.data = "nc:interoperability.end";
	pub_node_select.publish(node_feedback);
	ros::shutdown();
	// if flightmode is HOLD, continue code

}

void nodeSelectCB(const std_msgs::String& msg){

	node_status.data = msg.data;
}


void imageProcessingCB(const auvsi16::ImageProcessing& msg){

	center_buoy_x	= msg.center_buoy_x;
	center_buoy_y = msg.center_buoy_y;
	buoy_area			= msg.buoy_area;
	radius_buoy		= msg.radius_buoy;
	buoy_number  	= msg.buoy_number;
	imgproc_status = msg.detection_status;
}

void stateCB(const mavros_msgs::State& msg){
	state = msg.mode;
}

bool checkAUTOCruise(){

	if(state == "AUTO"){

		return true;
	}

	else{

		return false;
	}

}
