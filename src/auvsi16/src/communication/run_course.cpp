#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/NavSatFix.h"
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "../../include/auvsi16/auvsicommunication.hpp"

using namespace std;


void nodeSelectCB(const std_msgs::String& msg);
std_msgs::Int32 				heartbeat_status;
string	server_ip					=	"127.0.0.1";
int 		server_port				=	80;
string 	team_code					=	"rooster.php";
string	course_type 			= "courseA";
ros::Publisher pub_run_status;
ros::Publisher pub_node_select;
StartEndRunMessage auvsi_protocol(server_ip, server_port, course_type, team_code);

int main(int argc, char **argv){

	ros::init(argc, argv, "run_course");
	ros::NodeHandle run_course_nh;

	run_course_nh.getParam("/auvsi16/server_ip", server_ip);
	run_course_nh.getParam("/auvsi16/server_port", server_port);
	run_course_nh.getParam("/auvsi16/team_code", team_code);
	run_course_nh.getParam("/auvsi16/course_type", course_type);
	auvsi_protocol.setRemoteTarget(server_ip, server_port);
	auvsi_protocol.setTeamCode(team_code);
	auvsi_protocol.setCourseType(course_type);


	pub_run_status		= run_course_nh.advertise<std_msgs::Int32>("/auvsi16/course/status", 16);
	pub_node_select = run_course_nh.advertise<std_msgs::String>("/auvsi16/node/select", 16);
	ros::Subscriber sub_node_select 			= run_course_nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);

	ROS_INFO("Starting Run Course.");

	ros::spin();
	return 0;
}

void nodeSelectCB(const std_msgs::String& msg){

	ROS_INFO_STREAM("[RC] Receive command : "<< msg.data);
	std_msgs::Int32 response_status;
	std_msgs::String node_select;
	if (msg.data.compare("start_run") == 0){
		auvsi_protocol.setPayloadCommunication(communicationType::start_run);
		//cout << auvsi_protocol.getPayload();
		ROS_INFO_STREAM("\n\n[RC] Sending\n\n" << auvsi_protocol.getPayload());
		auvsi_protocol.sendTCP();
		ROS_INFO_STREAM("[RC] Receiving\n\n" << auvsi_protocol.getResponse());
		response_status.data = auvsi_protocol.decodeResponeStatus();
		pub_run_status.publish(response_status);

		if(response_status.data == 200){
			node_select.data = "nc:node_select.start_run.ok";
			pub_node_select.publish(node_select);
		}
		else{
			node_select.data = "nc:node_select.start_run.error";
			pub_node_select.publish(node_select);
		}

		//cout << auvsi_protocol.getRespone();

		ROS_INFO_STREAM("[RC] Response code : "<< "start_run." << response_status.data);
	}

	else if (msg.data.compare("end_run") == 0){
		auvsi_protocol.setPayloadCommunication(communicationType::end_run);
		//cout << auvsi_protocol.getPayload();
		ROS_INFO_STREAM("\n\n[RC] Sending\n\n" << auvsi_protocol.getPayload());
		auvsi_protocol.sendTCP();
		ROS_INFO_STREAM("[RC] Receiving\n\n" << auvsi_protocol.getResponse());
		response_status.data = auvsi_protocol.decodeResponeStatus();
		pub_run_status.publish(response_status);
		if(response_status.data == 200){
			node_select.data = "nc:node_select.end_run.ok";
			pub_node_select.publish(node_select);
		}
		else{
			node_select.data = "nc:node_select.start_run.error";
			pub_node_select.publish(node_select);
		}

		//cout << auvsi_protocol.getRespone();

		ROS_INFO_STREAM("[RC] Response code : "<< "end_run." << response_status.data);
	}

}
