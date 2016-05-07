#include "ros/ros.h"
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/WaypointSetCurrent.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"
#include "std_msgs/Float64.h"
#include "auvsi16/waypointSet.h"
#include <math.h>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace GeographicLib;


void calculateCoordinate(const double lat0, const double lon0, const  double h0,
						 double *lat_target,double *lon_target,
						 double x_target, double y_target);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission_clear");
  ros::NodeHandle n;
  
  ros::ServiceClient client_wp_set = n.serviceClient<auvsi16::waypointSet>("/auvsi16/waypoint_set_server");
  auvsi16::waypointSet wp_set;
	
  std_msgs::Float64 origin_lat;
  std_msgs::Float64 origin_long;
  origin_lat.data = -6.36248605092425;
  origin_long.data = 106.82503312826157;
  
  /*
  double latitude;
  double longitude;
  calculateCoordinate(origin_lat.data, origin_long.data, 73, &latitude,&longitude,30,0);
  

  wp_set.request.x_lat = latitude;
  wp_set.request.y_long = longitude;
  bool success_set = client_wp_set.call(wp_set);

	// Check for success and use the response .
	if(success_set){
		ROS_INFO_STREAM( "SC Success" ) ;
	} 
	else {
		ROS_ERROR_STREAM( "Failed to SC." ) ;
	}
	*/
	

  return 0;
}

void calculateCoordinate(const double lat0, const double lon0, const double h0,
						 double *lat_target,double *lon_target,
						 double x_target, double y_target){
  //const double lat0 = -6.36248605092425, lon0 = 106.82503312826157; // current location of boat
  
  Utility::set_digits();
  Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
  LocalCartesian proj(lat0, lon0, h0, earth);
  double h_target; // target location
      
  // Sample reverse calculation
  proj.Reverse(x_target, y_target, 0, *lat_target, *lon_target, h_target);
  
}
