#include "../../include/auvsi16/basic_mission_function.hpp"
#include "../../include/auvsi16/auvsicommunication.hpp"
#include "../../include/auvsi16/auvsi16_general.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "mavros_msgs/State.h"
#include "auvsi16/ImageProcessing.h"
#include "auvsi16/HSVValue.h"
#include "mavros_msgs/VFR_HUD.h"
#include <boost/shared_ptr.hpp>

#define	 MIN_GROUND_SPEED 0.2

void 	setDockingWaypoint			(double x_dock_gps, double y_dock_gps, double distance_from_triangle, double distance_from_zero);
void 	initializeDockingMission(double distance_from_triangle, double distance_from_zero);
void nodeSelectCB							(const std_msgs::String& msg);
bool scanDock									(int distance_from_shape,int distance_to_dock,int distance_to_away,int detection_accuracy);
bool checkAUTOCruise					();
void stateCB									(const mavros_msgs::State& msg);
bool changeFlightModeDebug		(string fm);
mavros_msgs::WaypointList getSymbolWaypoint(string symbol, string symbol_color);
void setSymbolWaypoint				(string first_symbol, string first_symbol_color, string second_symbol, string second_symbol_color);
void vfrHUDCB									(const mavros_msgs::VFR_HUD& msg);
void checkGroundSpeed					();
void imageProcessingCB				(const auvsi16::ImageProcessing& msg);

double	ground_speed = 0;

bool											imgproc_status = false;
bool											dock_gps_status = false;
double 										docking_coordinates_latitude[3];
double										docking_coordinates_longitude[3];
mavros_msgs::Waypoint 		center_dock_gps;
mavros_msgs::WaypointList	scan_dock_waypoint_list;
std_msgs::String 					node_status;
std_msgs::String 					node_feedback;
string										state;

double heading_zero 		= 0;
double heading_triangle = 0;
double heading_one 			= 0;
double heading_two 			= 0;

int 		center_buoy_x	= 0;
int 		center_buoy_y = 0;
double	buoy_area			= 0;
double 	radius_buoy		= 0;
int 		buoy_number  	= 0;
boost::shared_ptr<ImageProcessingInterface> imgproc_interface;

string	server_ip					=	"127.0.0.1";
int 		server_port				=	80;
string 	team_code					=	"rooster.php";
string	course_type 			= "courseA";
DockingMessage get_symbol(server_ip, server_port, course_type, team_code);
string first_symbol;
string first_symbol_color;
string second_symbol;
string second_symbol_color;

int main(int argc, char **argv){
	ros::init(argc, argv, "docking_mission_control");
	ros::NodeHandle nh("~");

	imgproc_interface.reset(new ImageProcessingInterface());
	setBMFConfiguration(nh);
	nh.getParam("heading_zero", heading_zero);
	nh.getParam("center_dock_lat", center_dock_gps.x_lat);
	nh.getParam("center_dock_long", center_dock_gps.y_long);
	nh.getParam("/auvsi16/server_ip", server_ip);
	nh.getParam("/auvsi16/server_port", server_port);
	nh.getParam("/auvsi16/team_code", team_code);
	nh.getParam("/auvsi16/course_type", course_type);
	get_symbol.setRemoteTarget(server_ip, server_port);
	get_symbol.setTeamCode(team_code);
	get_symbol.setCourseType(course_type);

	ros::Subscriber 						sub_vfr_hud 		= nh.subscribe("/mavros/vfr_hud", 1, vfrHUDCB);
	ros::Subscriber 						sub_state 			= nh.subscribe("/mavros/state", 1, stateCB);

	ros::Publisher pub_node_select	= nh.advertise<std_msgs::String>("/auvsi16/node/select", 16,true);
	ros::Subscriber sub_node_select	= nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);
	ros::Subscriber sub_imgproc_data	= nh.subscribe("/auvsi16/node/image_processing/data", 10, imageProcessingCB);

	int distance_from_shape = 7;
	int distance_to_dock 		= 2;
	int distance_to_away		= 10;
	int detection_accuracy  = 15;

	ROS_WARN_STREAM("Waiting for docking mission selected.");
	while (ros::ok() && node_status.data != "dm:docking.start"){
		ros::spinOnce();
	}
	ROS_WARN_STREAM("Docking mission selected.");
	int response_status = 0 ;
	while (response_status != 200 && ros::ok()){
		get_symbol.setPayloadCommunication();
		ROS_INFO_STREAM("\n\n[RC] Sending\n\n" << get_symbol.getPayload());
		get_symbol.sendTCP();
		ROS_INFO_STREAM("[RC] Receiving\n\n" << get_symbol.getResponse());
		response_status = get_symbol.decodeResponeStatus();
	}
	get_symbol.decodedResponse();
	first_symbol				= get_symbol.getFirstSymbol();
	first_symbol_color	= get_symbol.getFirstSymbolColor();
	second_symbol				= get_symbol.getSecondSymbol();
	second_symbol_color = get_symbol.getSecondSymbolColor();

	initializeDockingMission(distance_from_shape,20);
	changeFlightModeDebug("AUTO");
	if (scanDock(distance_from_shape, distance_to_dock, distance_to_away, detection_accuracy)){
		ROS_WARN_STREAM("Scan Complete");
		setSymbolWaypoint(first_symbol, first_symbol_color, second_symbol, second_symbol_color);
		ROS_WARN_STREAM("Docking");
		changeFlightModeDebug("AUTO");
		sleep(5);
		while(checkAUTOCruise()) ros::spinOnce();
	}
	node_feedback.data = "nc:docking.end";
	pub_node_select.publish(node_feedback);
	ros::shutdown();
}

void initializeDockingMission(double distance_from_triangle, double distance_from_zero){
	imgproc_interface->configuration(2);
	imgproc_interface->setHSVRange(0,179,90,255,0,255);
	ROS_INFO("Waiting for Image Processing Node!");

	while (ros::ok() && !imgproc_status){
		ros::spinOnce();
	}
	ROS_INFO("Image Processing Node Oke!");

	ROS_INFO_STREAM( "Got Docking location\t\t: " << dock_gps_status);
	ROS_INFO_STREAM("Heading Towards Zero Point\t: " << heading_zero);
	ROS_INFO_STREAM("Docking Latitude\t\t\t: " << center_dock_gps.x_lat);
	ROS_INFO_STREAM("Docking Longitude\t\t\t: " << center_dock_gps.y_long);

	setDockingWaypoint(center_dock_gps.x_lat, center_dock_gps.y_long, distance_from_triangle, distance_from_zero);
}


bool scanDock(int distance_from_shape,int distance_to_dock,int distance_to_away,int detection_accuracy){
	int while_counter = 0;
	bool main_scan = true;
	while (main_scan){
		while_counter++;
		bool first_cross_status		= false;
		bool triangle_status			= false;
		bool second_cross_status	= false;

		wp_sender->clearWaypointList();	// clear waypoint list
		imgproc_interface->configuration(2);
		ros::spinOnce();
		while(ros::ok() && !first_cross_status && checkAUTOCruise()){
			checkGroundSpeed();
			ros::spinOnce();
			if (areaCheck(center_buoy_x, 640, detection_accuracy)){
				moveToHeading(distance_to_away, heading_zero);
				moveToHeading(distance_to_dock, heading_triangle);
				first_cross_status = true;
				ROS_WARN_STREAM("First Cross Set");
			}
		}

		imgproc_interface->configuration(0);
		ros::spinOnce();
		imgproc_interface->configuration(1);
		while(ros::ok() && !triangle_status && checkAUTOCruise()){
			checkGroundSpeed();
			ros::spinOnce();
			if (areaCheck(center_buoy_x, 640, detection_accuracy)){
				moveToHeading(distance_to_away, heading_zero);
				moveToHeading(distance_to_dock, heading_triangle);
				triangle_status = true;
				ROS_WARN_STREAM("Triangle Set");
			}
		}


		imgproc_interface->configuration(0);
		ros::spinOnce();
		imgproc_interface->configuration(2);
		while(ros::ok() && !second_cross_status && checkAUTOCruise()){
			checkGroundSpeed();
			ros::spinOnce();
			if (areaCheck(center_buoy_x, 640, detection_accuracy)){
				moveToHeading(distance_to_away, heading_zero);
				moveToHeading(distance_to_dock, heading_triangle);
				second_cross_status = true;
				ROS_WARN_STREAM("Second Cross Set");
			}
		}

		if(first_cross_status && triangle_status && second_cross_status){
			main_scan = false;
			ROS_WARN_STREAM("All Symbol Recognized");
			return true;
		}
		else{
			wp_sender->clearWaypointList();	// clear waypoint list
			wp_sender->setWaypointList(scan_dock_waypoint_list);
			wp_sender->sendWaypointList();
			changeFlightModeDebug("AUTO");
			ROS_WARN_STREAM("Symbol Recognizing Failed");
			if (while_counter >= 2) {
				ROS_WARN_STREAM("Docking failed, changing to next mission");
				return false;
			}
			sleep(10);
		}

	}
}

void setDockingWaypoint(double x_dock_gps, double y_dock_gps, double distance_from_triangle, double distance_from_zero){

	/*						*(1)
	*
	* 	   C#
	*
	*
	* 	   T# 		*(0)
	*
	*
	* 	   C#
	*
	*							*(2)
	*
	* heading towards (0) from T#
	*/

	/*	*(2)
	*
	* 					   C#
	*
	*
	* 	*(0)			 T#
	*
	*
	* 					   C#
	*
	*		*(1)
	*
	* heading towards (0) from T#
	*/

	heading_triangle = heading_zero + 180;
	if (heading_triangle > 360) heading_triangle = heading_triangle - 360;

	heading_one = heading_zero - 90;
	if (heading_one < 0) heading_one = 360 + heading_one;

	heading_two = heading_zero + 90;
	if (heading_two > 360) heading_two = heading_two - 360;

	long double x_target;
	long double y_target;;

	positionEstimation(distance_from_triangle,heading_zero,&x_target, &y_target);
	calculateCoordinate(x_dock_gps, y_dock_gps, 0, &docking_coordinates_latitude[0],&docking_coordinates_longitude[0],x_target,y_target);

	positionEstimation(distance_from_zero,heading_one,&x_target, &y_target);
	calculateCoordinate(docking_coordinates_latitude[0], docking_coordinates_longitude[0], 0, &docking_coordinates_latitude[1],&docking_coordinates_longitude[1],x_target,y_target);

	positionEstimation(distance_from_zero,heading_two,&x_target, &y_target);
	calculateCoordinate(docking_coordinates_latitude[0], docking_coordinates_longitude[0], 0, &docking_coordinates_latitude[2],&docking_coordinates_longitude[2],x_target,y_target);

	wp_sender->clearWaypointList();
	wp_sender->addWaypoint(docking_coordinates_latitude[1], docking_coordinates_longitude[1]);
	wp_sender->addWaypoint(docking_coordinates_latitude[0], docking_coordinates_longitude[0]);
	wp_sender->addWaypoint(docking_coordinates_latitude[2], docking_coordinates_longitude[2]);
	scan_dock_waypoint_list = wp_sender->getWaypointList();		// save coordinates
	wp_sender->sendWaypointList();
}

void 	vfrHUDCB	(const mavros_msgs::VFR_HUD& msg){

	ground_speed = msg.groundspeed;
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

bool changeFlightModeDebug(string fm){
	if(changeFlightMode(fm.c_str())){
		ROS_WARN_STREAM("Changed to " << fm);
	}
	else {

		ROS_ERROR_STREAM("Failed changing to " << fm);
	}
}

mavros_msgs::WaypointList getSymbolWaypoint(string symbol, string symbol_color){
	mavros_msgs::WaypointList symbol_waypoint;
	if(course_type == "courseA"){
		if(symbol == "cruciform" && symbol_color == "green"){
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(1));
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(2));
			return symbol_waypoint;
		}

		else if(symbol == "triangle" && symbol_color == "blue"){
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(3));
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(4));
			return symbol_waypoint;
		}
		else if(symbol == "cruciform" && symbol_color == "red"){
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(5));
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(6));
			return symbol_waypoint;
		}
	}
	else if(course_type == "courseB"){
		if(symbol == "cruciform" && symbol_color == "green"){
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(5));
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(6));
			return symbol_waypoint;
		}

		else if(symbol == "triangle" && symbol_color == "blue"){
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(3));
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(4));
			return symbol_waypoint;
		}
		else if(symbol == "cruciform" && symbol_color == "red"){
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(1));
			symbol_waypoint.waypoints.push_back(wp_sender->getWaypointList().waypoints.at(2));
			return symbol_waypoint;
		}
	}
	return symbol_waypoint;
}

void setSymbolWaypoint(string first_symbol,string first_symbol_color,string second_symbol,string second_symbol_color){
	mavros_msgs::WaypointList docking_sequence;
	ROS_WARN_STREAM("Setting First Symbol");
	docking_sequence.waypoints.push_back(getSymbolWaypoint(first_symbol, first_symbol_color).waypoints.at(0));
	docking_sequence.waypoints.push_back(getSymbolWaypoint(first_symbol, first_symbol_color).waypoints.at(1));
	ROS_WARN_STREAM("Setting Second Symbol");
	docking_sequence.waypoints.push_back(getSymbolWaypoint(second_symbol, second_symbol_color).waypoints.at(0));
	docking_sequence.waypoints.push_back(getSymbolWaypoint(second_symbol, second_symbol_color).waypoints.at(1));
	wp_sender->clearWaypointList();
	wp_sender->addWaypoint(docking_sequence);
	wp_sender->sendWaypointList();
	ROS_WARN_STREAM("Setting waypoint success");
}

void checkGroundSpeed(){
	if(ground_speed < MIN_GROUND_SPEED && checkAUTOCruise() ){
		changeFlightModeDebug("MANUAL");
		sleep(5);
		changeFlightModeDebug("AUTO");
		sleep(10);
	}
}
