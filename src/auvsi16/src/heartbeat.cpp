#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/NavSatFix.h"
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "../include/auvsi16/auvsicommunication.hpp"

using namespace std;
void challengeCB(const std_msgs::String& msg);

void globalPositionCB(const sensor_msgs::NavSatFix& msg);

sensor_msgs::NavSatFix	global_position;
std_msgs::Int32 				heartbeat_status;
string	challenge_status	= "idle";
string	server_ip					=	"127.0.0.1";
int 		server_port				=	80;
string 	team_code					=	"rooster.php";
string	course_type 			= "courseA";

int main(int argc, char **argv){
	getTime time_lord;
	ros::init(argc, argv, "heartbeat");
	ros::NodeHandle heartbeat_nh;

	heartbeat_nh.getParam("/auvsi16/server_ip", server_ip);
	heartbeat_nh.getParam("/auvsi16/server_port", server_port);
	heartbeat_nh.getParam("/auvsi16/team_code", team_code);
	heartbeat_nh.getParam("/auvsi16/course_type", course_type);

	ros::Publisher pub_heartbeat_status = heartbeat_nh.advertise<std_msgs::Int32>("/auvsi16/heartbeat/status", 16);

	ros::Subscriber sub_challange 			= heartbeat_nh.subscribe("/auvsi16/challenge", 10, challengeCB);
	ros::Subscriber sub_global_position = heartbeat_nh.subscribe("/mavros/global_position/global", 1, globalPositionCB);
	HeartbeatMessage auvsi_protocol(server_ip, server_port, course_type, team_code);
	ros::Rate loop_rate(1);

	/*	Do not wait for data, always sent data no matter what the data is.
	while(ros::ok() && (challenge_status.compare("idle") == 0 || global_position.latitude == 0 || global_position.longitude == 0)){
		ros::spinOnce();
	}
	*/

	ROS_INFO("Starting Heartbeat.");

	while(ros::ok()){
		auvsi_protocol.setPayloadCommunication(time_lord.getYMDHS(), challenge_status, global_position.latitude,global_position.longitude);
		auvsi_protocol.sendTCP();
		//cout << auvsi_protocol.getRespone();
		heartbeat_status.data = auvsi_protocol.decodeResponeStatus();
		ROS_INFO_STREAM("Response code : "<<heartbeat_status.data);
		pub_heartbeat_status.publish(heartbeat_status);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void challengeCB(const std_msgs::String& msg){
	challenge_status = msg.data;
}

void globalPositionCB(const sensor_msgs::NavSatFix& msg){

	global_position.latitude = msg.latitude;
	global_position.longitude = msg.longitude;
	global_position.altitude = msg.altitude;
}
