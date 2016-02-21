#include "ros/ros.h"

#include "mavros_msgs/OverrideRCIn.h"
#include "mavros_msgs/RCIn.h"
#include <string>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

int rc_failsafe_override_flag = 0;
int rc_in_data_channel[8];
int channel_7_mid = 1500;
// channel_7_off = 987 | channel_7_on = 2010


void rcinReceiver(const mavros_msgs::RCIn& rc_in_data);

ros::Publisher pub_rc_override;
ros::Publisher pub_override_status;
mavros_msgs::OverrideRCIn rc_override_data;
std_msgs::Bool override_data;

int main(int argc, char **argv){

	ros::init(argc, argv, "system_controller");
	ros::NodeHandle sys_ctrl;
	ros::Subscriber rc_in_sub 	= sys_ctrl.subscribe("/mavros/rc/in", 1, rcinReceiver);
	pub_override_status 		= sys_ctrl.advertise<std_msgs::Bool>("auvsi16/override_status", 1);
	pub_rc_override				= sys_ctrl.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1, true);
	ROS_INFO("Starting System Monitor.");
	// set initial state for rc override take over.
	if(rc_in_data_channel[6] < channel_7_mid){
			rc_failsafe_override_flag = 0;
	}
	else if (rc_in_data_channel[6] > channel_7_mid){
			rc_failsafe_override_flag = 1;
	}
	
	ros::spin();
  
  return 0;
}

void rcinReceiver(const mavros_msgs::RCIn& rc_in_data){
	int x;
	for (x = 0; x<8;x++){
		rc_in_data_channel[x] = rc_in_data.channels[x];
	}
	
	if(rc_in_data_channel[6] < channel_7_mid && rc_failsafe_override_flag == 0){
		for(int i=0; i < 8; i++) rc_override_data.channels[i] = 0;
		
		pub_rc_override.publish(rc_override_data);
		rc_failsafe_override_flag = 1;
		ROS_ERROR_STREAM( "[NC] RC is now taking over!") ;
	}
	
	else if (rc_in_data_channel[6] > channel_7_mid && rc_failsafe_override_flag == 1){
		rc_failsafe_override_flag = 0;
		ROS_ERROR_STREAM( "[NC] Drone is now taking over") ;
	}
	
}
