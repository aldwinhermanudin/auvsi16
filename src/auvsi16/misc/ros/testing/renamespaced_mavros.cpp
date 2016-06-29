#include "ros/ros.h"
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/WaypointSetCurrent.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/SetMode.h"
#include "std_msgs/Float64.h"
#include <math.h>


using namespace std;

bool changeFlightMode(const char* flight_mode);
bool changeFlightModeDebug(string fm);
ros::ServiceClient client_set_flightmode;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "renamespaced_mavros");
  ros::NodeHandle nh;
  
  client_set_flightmode 	= nh.serviceClient<mavros_msgs::SetMode>("/robosub/mavros/set_mode");
  changeFlightModeDebug("AUTO");
  return 0;
}


bool changeFlightMode(const char* flight_mode){
	mavros_msgs::SetMode flightmode;
	flightmode.request.base_mode = 0;				//Set to 0 to use custom_mode
	flightmode.request.custom_mode = flight_mode;		//Set to '' to use base_mode
	bool success = client_set_flightmode.call(flightmode);

	// Check for success and print out info.
	if(success){
		return true;
		//ROS_INFO_STREAM( "Flight Mode changed to "<< flight.request.custom_mode ) ;
	}
	else {
		return  false;
		//ROS_ERROR_STREAM( "Failed to changed." ) ;
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
