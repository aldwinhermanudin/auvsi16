#include "ros/ros.h"
#include <string>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "../include/auvsi16/auvsicommunication.hpp"

using namespace std;

int main(int argc, char **argv){

	ros::init(argc, argv, "heartbeat");
	ros::NodeHandle heartbeat;

	ROS_INFO("Starting Heartbeat.");

	double secs =ros::Time::now().toSec();

	cout << secs;

	ros::spinOnce();



  return 0;
}
