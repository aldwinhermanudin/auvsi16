#include "ros/ros.h"
#include "mavros_msgs/OverrideRCIn.h"
#include "auvsi16/overrideMotorRC.h"

#define STEERING 0
#define THROTTLE 2

ros::Publisher pub_override_rc;
mavros_msgs::OverrideRCIn override_out;
void overrideInputCB(const auvsi16::overrideMotorRC& override_recv);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_control");
  ros::NodeHandle n;

  ros::Publisher pub_override_rc = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
  ros::Subscriber sub_overrideMotorRC = n.subscribe("auvsi16/overrideMotorRC", 10, overrideInputCB);
  
  ros::spin();
  return 0;
}

void overrideInputCB(const auvsi16::overrideMotorRC& override_recv){
   
  if(override_recv.overrideRC){
	for(int i=0; i < 8; i++) override_out.channels[i] = 0;	//Releases all Channels First
	
	override_out.channels[THROTTLE] = override_recv.throttle;
    override_out.channels[STEERING] = override_recv.steering;
    pub_override_rc.publish(override_out);
  }
  
  else if(!override_recv.overrideRC){
	for(int i=0; i < 8; i++) override_out.channels[i] = 0;	//Releases all Channels
	
    pub_override_rc.publish(override_out);
  }
}
