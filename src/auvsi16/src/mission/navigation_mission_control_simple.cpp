#include "../../include/auvsi16/basic_mission_function.hpp"
#include "../../include/auvsi16/auvsi16_general.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <auvsi16/ImageProcessing.h>
#include "mavros_msgs/State.h"

#define	 MIN_GROUND_SPEED 0.2

void nodeSelectCB(const std_msgs::String& msg);
void imageProcessingCB				(const auvsi16::ImageProcessing& msg);
void stateCB									(const mavros_msgs::State& msg);
bool checkAUTOCruise					();
void checkGroundSpeed					();
bool changeFlightModeDebug		(string fm);
void vfrHUDCB									(const mavros_msgs::VFR_HUD& msg);

ros::Publisher pub_imgproc_select;

double	ground_speed = 0;
bool		imgproc_status = false;
int 		center_buoy_x	= 0;
int 		center_buoy_y = 0;
double	buoy_area			= 0;
double 	radius_buoy		= 0;
int 		buoy_number  	= 0;
int 	arena_distance	= 46;

std_msgs::String node_status;
std_msgs::String node_feedback;
string										state;
int main(int argc, char **argv){

	ros::init(argc, argv, "navigation_mission_simple");
	ros::NodeHandle nh("~");
	
	nh.getParam("arena_distance", arena_distance);
	image_transport::ImageTransport it(nh);
	setBMFConfiguration(nh);

	RoverAUTOConfiguration auto_speed_conf;

	changePID(0.3, 0, 0);

	ros::Subscriber 						sub_state 			= nh.subscribe("/mavros/state", 1, stateCB);
	ros::Subscriber 						sub_vfr_hud 		= nh.subscribe("/mavros/vfr_hud", 1, vfrHUDCB);

	ros::Publisher pub_run_status		= nh.advertise<std_msgs::String>("/auvsi16/mission/navigation/status", 16);
	ros::Publisher pub_node_select 	= nh.advertise<std_msgs::String>("/auvsi16/node/select", 16,true);
	ros::Subscriber sub_node_select = nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);

	ros::Subscriber sub_imgproc_data	= nh.subscribe("/auvsi16/node/image_processing/data", 10, imageProcessingCB);

	ROS_WARN_STREAM("Waiting for navigation mission selected.");
	while (ros::ok() && node_status.data != "nm:navigation.start"){
		ros::spinOnce();
	}

	auto_speed_conf.setFullSpeed();
	if(auto_speed_conf.sendParameter()){
		ROS_INFO("Success change speed to Full power");
	}
	else{

			ROS_INFO("Failed change speed to Full power");
	}
	sleep(4);
	
	ROS_WARN_STREAM("Navigation mission selected.");

	ros::spinOnce();	// read frame
	double compass_hdg_at_capture = compass_hdg;
	ROS_INFO_STREAM("Compass Heading : " << compass_hdg);
	ROS_INFO_STREAM("Latitude : " << global_position.latitude);
	ROS_INFO_STREAM("Longitude : " <<  global_position.longitude);
	ROS_INFO_STREAM("X Position : " << center_buoy_x);
	//ROS_WARN_STREAM("Detected Red Buoy : " << red_buoy);
	compass_hdg = compass_hdg_at_capture - 5;
	if (compass_hdg < 0){
		compass_hdg = 360 + compass_hdg;
	}
	else if (compass_hdg > 360){
		compass_hdg = compass_hdg - 360;
	}
	ROS_INFO_STREAM("New Compass Heading: " << compass_hdg);

	if(moveForward(arena_distance)){
		ROS_WARN_STREAM("Set Waypoint Success!");
	}
	else {
		ROS_WARN_STREAM("Set Waypoint Failed!");
	}

	ROS_WARN_STREAM("Destination Set!");
	if(changeFlightMode("AUTO")){
		ROS_WARN_STREAM("Changed to AUTO!");
	}
	else {

		ROS_ERROR_STREAM("Failed changing to AUTO!");
	}
	
	sleep(1);
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	while(checkAUTOCruise()) {
		checkGroundSpeed();
		ros::spinOnce();
	}
	
	pub_node_select.publish(node_feedback);
	auto_speed_conf.setNormalSpeed();
	if(auto_speed_conf.sendParameter()){
		ROS_INFO("Success change speed to Normal power");
	}
	else{

			ROS_INFO("Failed change speed to Normal power");
	}
	sleep(4);
	node_feedback.data = "nc:navigation.end";
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

void 	vfrHUDCB	(const mavros_msgs::VFR_HUD& msg){

	ground_speed = msg.groundspeed;
}

bool checkAUTOCruise(){

	if(state == "AUTO"){

		return true;
	}

	else{

		return false;
	}

}
void checkGroundSpeed(){
	if(ground_speed < MIN_GROUND_SPEED && checkAUTOCruise() ){
		changeFlightModeDebug("MANUAL");
		sleep(5);
		changeFlightModeDebug("AUTO");
		sleep(5);
	}
}


bool changeFlightModeDebug(string fm){
	if(changeFlightMode(fm.c_str())){
		ROS_WARN_STREAM("Changed to " << fm);
	}
	else {

		ROS_ERROR_STREAM("Failed changing to " << fm);
	}
}
