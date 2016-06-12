#include <ros/ros.h>
#include "std_msgs/String.h"
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "../include/auvsi16/auvsicommunication.hpp"

using namespace std;

void nodeSelectCB(const std_msgs::String& msg);
	std_msgs::String node_select;

	int main(int argc, char **argv){

	ros::init(argc, argv, "node_controller");
	ros::NodeHandle nh;

	ros::Publisher pub_node_select  = nh.advertise<std_msgs::String>("/auvsi16/node/select", 16);
	ros::Subscriber sub_node_select	= nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);


		ROS_INFO_STREAM("Sending Run Course Command.");
		node_select.data = "start_run";
		pub_node_select.publish(node_select);

		while(ros::ok() && node_select.data.compare("nc:node_select.start_run.ok") != 0) ros::spinOnce();

		ROS_INFO_STREAM("Launching Navigation Mission.");
		node_select.data = "nm:navigation.start";
		pub_node_select.publish(node_select);

		while(ros::ok() && node_select.data.compare("nc:navigation.end") != 0) ros::spinOnce();

		ROS_INFO_STREAM("Launching Docking Mission.");
		node_select.data = "nc:docking.start";
		pub_node_select.publish(node_select);

		while(ros::ok() && node_select.data.compare("nc:docking.end") != 0) ros::spinOnce();

		ROS_INFO_STREAM("Launching Interoperability Mission.");
		node_select.data = "nc:interoperability.start";
		pub_node_select.publish(node_select);

		while(ros::ok() && node_select.data.compare("nc:interoperability.end") != 0) ros::spinOnce();

		ROS_INFO_STREAM("Ending Course.");
		node_select.data = "end_run";
		pub_node_select.publish(node_select);

		while(ros::ok() && node_select.data.compare("nc:node_select.end_run.ok") != 0) ros::spinOnce();

}

void nodeSelectCB(const std_msgs::String& msg){
	node_select.data = msg.data;
}
