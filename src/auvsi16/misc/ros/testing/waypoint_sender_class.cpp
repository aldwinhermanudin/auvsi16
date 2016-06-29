#include "ros/ros.h"
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/WaypointSetCurrent.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/Waypoint.h"
#include "std_msgs/Float64.h"
#include <boost/shared_ptr.hpp>
#include <math.h>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "../../../include/auvsi16/waypoint_sender.hpp"

using namespace std;
using namespace GeographicLib;
boost::shared_ptr<WaypointSender> wp_sender;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mission_clear");
  ros::NodeHandle n;

  wp_sender.reset(new WaypointSender());

  mavros_msgs::Waypoint waypoint_input;
	waypoint_input.frame = 3;
	waypoint_input.command = 16;
	waypoint_input.is_current = true;
	waypoint_input.autocontinue = true;
	waypoint_input.param1 = 0;
	waypoint_input.param2 = 0;
	waypoint_input.param3 = 0;
	waypoint_input.param4 = 0;
	waypoint_input.x_lat = 129.123;
	waypoint_input.y_long = -102.213;
	waypoint_input.z_alt = 0;

	wp_sender->addWaypoint(waypoint_input);
	wp_sender->addWaypoint(waypoint_input);

  cout << wp_sender->sendWaypointList() << endl;

  return 0;
}
