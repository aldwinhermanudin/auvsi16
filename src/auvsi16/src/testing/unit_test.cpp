#include "../../include/auvsi16/basic_mission_function.hpp"

int main(int argc, char **argv){
	ros::init(argc, argv, "unit_test");
	ros::NodeHandle nh;	
	setBMFConfiguration(nh);
	
	long double x;
	long double y;
	
	positionEstimation(10,135,&x,&y);
	
	int x_int = x;
	int y_int = y;
	
	cout << x << endl;
	cout << y << endl;
	cout << endl;
	cout << x_int << endl;
	cout << y_int << endl;
	
	double origin_lat = -6.36248605092425;
    double origin_long = 106.82503312826157;
	
	clearWaypointList();
	
	for(int i = 0; i <5; i++){
		addWaypoint(origin_lat--, origin_long++);
		// Check for success and use the response .
		
	}
	
	bool success_set = sendWaypointList();
	if(success_set){
			ROS_INFO_STREAM( "SC Success" ) ;
		} 
	else {
			ROS_ERROR_STREAM( "Failed to SC." ) ;
		}
}
