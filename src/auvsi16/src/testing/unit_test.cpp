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
	ros::shutdown();	

}
