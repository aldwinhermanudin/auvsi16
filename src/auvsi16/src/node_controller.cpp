#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "../include/auvsi16/auvsicommunication.hpp"

using namespace std;

void nodeSelectCB(const std_msgs::String& msg);
	std_msgs::String node_select;
	std_msgs::String challenge_status;

	int main(int argc, char **argv){

	ros::init(argc, argv, "node_controller");
	ros::NodeHandle nh;

	ros::Publisher pub_node_select  = nh.advertise<std_msgs::String>("/auvsi16/node/select", 16);
	ros::Publisher pub_challenge  = nh.advertise<std_msgs::String>("/auvsi16/challenge", 16, true);
	ros::Subscriber sub_node_select	= nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);


		ROS_INFO_STREAM("Sending Run Course Command in 5 seconds");
		sleep(5);
		node_select.data = "start_run";
		pub_node_select.publish(node_select);

		while(ros::ok() && node_select.data != "nc:node_select.start_run.ok") ros::spinOnce();

		ROS_INFO_STREAM("Launching Navigation Mission in 5 seconds");
		sleep(5);

		node_select.data = "nm:navigation.start";
		pub_node_select.publish(node_select);
		challenge_status.data = "gates";
		pub_challenge.publish(challenge_status);

		while(ros::ok() && node_select.data != "nc:navigation.end") ros::spinOnce();

		ROS_INFO_STREAM("Launching Obstacle Mission in 5 seconds");
		sleep(5);
		node_select.data = "om:obstacle.start";
		pub_node_select.publish(node_select);
		challenge_status.data = "obstacle";
		pub_challenge.publish(challenge_status);


		while(ros::ok() && node_select.data != "nc:obstacle.end") ros::spinOnce();

		ROS_INFO_STREAM("Launching Docking Mission in 5 seconds");
		sleep(5);
		node_select.data = "dm:docking.start";
		pub_node_select.publish(node_select);
		challenge_status.data = "docking";
		pub_challenge.publish(challenge_status);

		while(ros::ok() && node_select.data != "nc:docking.end") ros::spinOnce();

		ROS_INFO_STREAM("Ending Course in 5 seconds");
		sleep(5);
		node_select.data = "end_run";
		pub_node_select.publish(node_select);
		challenge_status.data = "return";
		pub_challenge.publish(challenge_status);

		while(ros::ok() && node_select.data != "nc:node_select.end_run.ok") ros::spinOnce();

}

void nodeSelectCB(const std_msgs::String& msg){
	node_select.data = msg.data;
}
