#include "../../include/auvsi16/basic_mission_function.hpp"
#include "../../include/auvsi16/auvsi16_general.hpp"
#include "../../include/auvsi16/auvsicommunication.hpp"
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

std_msgs::String node_status;
std_msgs::String node_feedback;
string										state;

double  obstacle_heading = 0;
string	server_ip					=	"127.0.0.1";
int 		server_port				=	80;
string 	team_code					=	"rooster.php";
string	course_type 			= "courseA";
ObstacleMessage get_gate(server_ip, server_port, course_type, team_code);

int main(int argc, char **argv){

	ros::init(argc, argv, "obstacle_mission_simple");
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);
	setBMFConfiguration(nh);
	nh.getParam("obstacle_heading", obstacle_heading);
	nh.getParam("/auvsi16/server_ip", server_ip);
	nh.getParam("/auvsi16/server_port", server_port);
	nh.getParam("/auvsi16/team_code", team_code);
	nh.getParam("/auvsi16/course_type", course_type);
	get_gate.setRemoteTarget(server_ip, server_port);
	get_gate.setTeamCode(team_code);
	get_gate.setCourseType(course_type);

	ros::Subscriber 						sub_vfr_hud 		= nh.subscribe("/mavros/vfr_hud", 1, vfrHUDCB);
		ros::Subscriber 						sub_state 			= nh.subscribe("/mavros/state", 1, stateCB);

	ros::Publisher pub_run_status		= nh.advertise<std_msgs::String>("/auvsi16/obstacle/navigation/status", 16);
	ros::Publisher pub_node_select 	= nh.advertise<std_msgs::String>("/auvsi16/node/select", 16,true);
	ros::Subscriber sub_node_select = nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);

	ros::Subscriber sub_imgproc_data	= nh.subscribe("/auvsi16/node/image_processing/data", 10, imageProcessingCB);

	ROS_WARN_STREAM("Waiting for obstacle mission selected.");
	while (ros::ok() && node_status.data != "om:obstacle.start"){
		ros::spinOnce();
	}
	ROS_WARN_STREAM("Obstacle mission selected.");

	ROS_INFO("Waiting for Image Processing Node!");
	// this it to whether image_received is empty or not, move this to a function.
	// while (ros::ok() && front_image.empty() && right_image.empty()){
	while (ros::ok() && !imgproc_status){
		ros::spinOnce();
	}

	int response_status = 0 ;
	while (response_status != 200 && ros::ok()){
		get_gate.setPayloadCommunication();
		ROS_INFO_STREAM("\n\n[RC] Sending\n\n" << get_gate.getPayload());
		get_gate.sendTCP();
		ROS_INFO_STREAM("[RC] Receiving\n\n" << get_gate.getResponse());
		response_status = get_gate.decodeResponeStatus();
	}

	ros::spinOnce();	// read frame
	ROS_INFO_STREAM("Obstacle Heading : " << obstacle_heading);
	ROS_INFO_STREAM("Latitude : " << global_position.latitude);
	ROS_INFO_STREAM("Longitude : " <<  global_position.longitude);
	//ROS_WARN_STREAM("Detected Red Buoy : " << red_buoy);

	compass_hdg = obstacle_heading;
	if(moveForward(40)){
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
	node_feedback.data = "nc:obstacle.end";
	pub_node_select.publish(node_feedback);
	ros::shutdown();
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
		sleep(10);
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
