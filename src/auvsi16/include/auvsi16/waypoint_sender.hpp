// ########################################## start of WaypointSender class ########################################## //
#include "ros/ros.h"
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/WaypointSetCurrent.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointList.h"

class WaypointSender {

private:
  ros::ServiceClient client_wp_clear;
  ros::ServiceClient client_wp_push;
  ros::ServiceClient client_wp_set_current;
  mavros_msgs::WaypointList 	waypoint_list;
  ros::NodeHandle nh;

public:
  WaypointSender          ();
  bool sendWaypointList   ();
  void setWaypointList    (mavros_msgs::WaypointList waypoint_list);
  mavros_msgs::WaypointList getWaypointList    ();
  void clearWaypointList  ();
  void addWaypoint        (double latitude, double longitude);
  void addWaypoint        (mavros_msgs::Waypoint waypoint_input);
  void addWaypoint        (mavros_msgs::WaypointList waypoint_input);

};

WaypointSender::WaypointSender(){
  client_wp_clear = nh.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
	client_wp_push = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
	client_wp_set_current = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("/mavros/mission/set_current");
  }

bool WaypointSender::sendWaypointList(){

    mavros_msgs::WaypointClear wp_clear;
    mavros_msgs::WaypointPush wp_push;
    mavros_msgs::WaypointSetCurrent wp_set_current;

    wp_push.request.waypoints = waypoint_list.waypoints;

    wp_set_current.request.wp_seq = 1;

    bool success_clear = client_wp_clear.call(wp_clear);
    bool success_push = client_wp_push.call(wp_push);
    bool success_current = client_wp_set_current.call(wp_set_current);


    if (success_clear && success_push && success_current){
      return true;
    } else {
      return false;
    }
}

void WaypointSender::setWaypointList(mavros_msgs::WaypointList waypoint_list){
  this->waypoint_list = waypoint_list;
}

mavros_msgs::WaypointList WaypointSender::getWaypointList (){
  return this->waypoint_list;
}

void WaypointSender::clearWaypointList(){

	mavros_msgs::Waypoint home_waypoint;
	home_waypoint.frame = 0;
	home_waypoint.command = 16;
	home_waypoint.is_current = false;
	home_waypoint.autocontinue = true;
	home_waypoint.param1 = 0;
	home_waypoint.param2 = 0;
	home_waypoint.param3 = 0;
	home_waypoint.param4 = 0;
	home_waypoint.x_lat = 0;
	home_waypoint.y_long = 0;
	home_waypoint.z_alt = 0;

	waypoint_list.waypoints.clear();
	waypoint_list.waypoints.push_back(home_waypoint);

}

void WaypointSender::addWaypoint(double latitude, double longitude){

	mavros_msgs::Waypoint waypoint_input;
	waypoint_input.frame = 3;
	waypoint_input.command = 16;
	waypoint_input.is_current = true;
	waypoint_input.autocontinue = true;
	waypoint_input.param1 = 0;
	waypoint_input.param2 = 0;
	waypoint_input.param3 = 0;
	waypoint_input.param4 = 0;
	waypoint_input.x_lat = latitude;
	waypoint_input.y_long = longitude;
	waypoint_input.z_alt = 0;

	waypoint_list.waypoints.push_back(waypoint_input);

}

void WaypointSender::addWaypoint(mavros_msgs::Waypoint waypoint_input){

	waypoint_list.waypoints.push_back(waypoint_input);
}

void WaypointSender::addWaypoint (mavros_msgs::WaypointList waypoint_input){
  for (std::vector<mavros_msgs::Waypoint>::iterator it = waypoint_input.waypoints.begin() ; it != waypoint_input.waypoints.end(); ++it)
    waypoint_list.waypoints.push_back(*it);
  
}
// ########################################## start of WaypointSender class ########################################## //
