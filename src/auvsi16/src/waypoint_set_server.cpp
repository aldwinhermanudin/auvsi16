#include "ros/ros.h"
#include "auvsi16/waypointSet.h"
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/WaypointSetCurrent.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"

ros::ServiceClient client_wp_clear;
ros::ServiceClient client_wp_push;
ros::ServiceClient client_wp_set_current;

bool waypointSet(auvsi16::waypointSet::Request &req, auvsi16::waypointSet::Response &res);

int main(int argc, char **argv) {
	ros::init(argc, argv, "waypoint_set_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/auvsi16/waypoint_set_server", waypointSet);
	client_wp_clear = n.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
	client_wp_push = n.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
	client_wp_set_current = n.serviceClient<mavros_msgs::WaypointSetCurrent>("/mavros/mission/set_current");

	ros::spin();
	return 0;
}

bool waypointSet(auvsi16::waypointSet::Request &req, auvsi16::waypointSet::Response &res) {
	mavros_msgs::WaypointClear wp_clear;
	mavros_msgs::WaypointPush wp_push;
	mavros_msgs::Waypoint targeted_waypoint;
	mavros_msgs::WaypointSetCurrent wp_set_current;

	targeted_waypoint.frame = 3;
	targeted_waypoint.command = 16;
	targeted_waypoint.is_current = true;
	targeted_waypoint.autocontinue = true;
	targeted_waypoint.param1 = 0;
	targeted_waypoint.param2 = 0;
	targeted_waypoint.param3 = 0;
	targeted_waypoint.param4 = 0;
	targeted_waypoint.x_lat = req.x_lat;
	targeted_waypoint.y_long = req.y_long;
	targeted_waypoint.z_alt = 0;
	wp_push.request.waypoints.clear();
	wp_push.request.waypoints.push_back(targeted_waypoint); //first waypoint will be overridden by fcu
	wp_push.request.waypoints.push_back(targeted_waypoint);
	
	wp_set_current.request.wp_seq = 1;	

	bool success_clear = client_wp_clear.call(wp_clear);
	bool success_push = client_wp_push.call(wp_push);
	bool success_current = client_wp_set_current.call(wp_set_current);

	
	if (success_clear && success_push && success_current){
		res.state = true;
		return true;
	} else {
		res.state = false;
		return false;
	}
}
