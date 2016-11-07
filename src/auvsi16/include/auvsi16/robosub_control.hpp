// ########################################## start of MavrosInterface class ########################################## //
#include "ros/ros.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "std_msgs/Bool.h"
#include <iostream>

#define STEERING 0
#define THROTTLE 2
#define MAX_THROTTLE 1942
#define MIN_THROTTLE 1101
#define MAX_STEERING 1942
#define MIN_STEERING 1116
#define IDLE_STATE 1500
using namespace std;

class RobosubControl {

protected:

  ros::NodeHandle nh;
  ros::Publisher pub_override_rc;

public:
  RobosubControl  ();
  void motorControl(int steering, int throttle);
  void releaseControl();
};

RobosubControl::RobosubControl(){
  pub_override_rc = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros_robosub/rc/override", 10);
	
}

void RobosubControl::motorControl(int steering, int throttle){
mavros_msgs::OverrideRCIn override_out;
  // Drone control
	for(int i=0; i < 8; i++) override_out.channels[i] = 0;	//Releases all Channels First

	if (throttle > MAX_THROTTLE){
		override_out.channels[THROTTLE] = MAX_THROTTLE;
	}
	else if (throttle < MIN_THROTTLE){
		override_out.channels[THROTTLE] = MIN_THROTTLE;
	}
	else {
		override_out.channels[THROTTLE] = throttle;
	}

	if (steering > MAX_STEERING){
		override_out.channels[STEERING] = MAX_STEERING;
	}
	else if (steering < MIN_STEERING){
		override_out.channels[STEERING] = MIN_STEERING;
	}
	else {
		override_out.channels[STEERING] = steering;
  	}

      pub_override_rc.publish(override_out);
 }
 
void RobosubControl::releaseControl(){
mavros_msgs::OverrideRCIn override_out;
  // Drone control
	for(int i=0; i < 8; i++) override_out.channels[i] = 0;	//Releases all Channels

      pub_override_rc.publish(override_out);
 }
 
// ########################################## start of MavrosInterface class ########################################## //
