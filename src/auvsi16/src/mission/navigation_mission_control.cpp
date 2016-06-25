#include "../../include/auvsi16/basic_mission_function.hpp"
#include "../../include/auvsi16/auvsi16_general.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <auvsi16/ImageProcessing.h>

void nodeSelectCB(const std_msgs::String& msg);
void imageProcessingCB				(const auvsi16::ImageProcessing& msg);

ros::Publisher pub_imgproc_select;

bool		imgproc_status = false;
int 		center_buoy_x	= 0;
int 		center_buoy_y = 0;
double	buoy_area			= 0;
double 	radius_buoy		= 0;
int 		buoy_number  	= 0;

std_msgs::String node_status;
std_msgs::String node_feedback;
int main(int argc, char **argv){

	ros::init(argc, argv, "navigation_mission");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	setBMFConfiguration(nh);

	ImageProcessingInterface imgproc_interface;

	changePID(0.3, 0, 0);

	ros::Publisher pub_run_status		= nh.advertise<std_msgs::String>("/auvsi16/mission/navigation/status", 16);
	ros::Publisher pub_node_select 	= nh.advertise<std_msgs::String>("/auvsi16/node/select", 16,true);
	ros::Subscriber sub_node_select = nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);

	ros::Subscriber sub_imgproc_data	= nh.subscribe("/auvsi16/node/image_processing/data", 10, imageProcessingCB);

	ROS_WARN_STREAM("Waiting for navigation mission selected.");
	while (ros::ok() && node_status.data != "nm:navigation.start"){
				ros::spinOnce();
	}
	imgproc_interface.setHSVRange(166,179,40,184,138,255);
	imgproc_interface.configuration(3);
	ROS_INFO("Waiting for Image Processing Node!");
	// this it to whether image_received is empty or not, move this to a function.
	// while (ros::ok() && front_image.empty() && right_image.empty()){
	while (ros::ok() && !imgproc_status){
		ros::spinOnce();
	}

	while (ros::ok()){

		ros::spinOnce();	// read frame
		double compass_hdg_at_capture = compass_hdg;
		ROS_INFO_STREAM("Compass Heading : " << compass_hdg);
		ROS_INFO_STREAM("Compass Heading Current: " << compass_hdg_at_capture);
		ROS_INFO_STREAM("Latitude : " << global_position.latitude);
		ROS_INFO_STREAM("Longitude : " <<  global_position.longitude);
		ROS_INFO_STREAM("X Position : " << center_buoy_x);
		//ROS_WARN_STREAM("Detected Red Buoy : " << red_buoy);
		if (areaCheck(center_buoy_x, 320, 2)){
			compass_hdg = compass_hdg_at_capture + 10;
			if (compass_hdg  > 360){
				compass_hdg = compass_hdg - 360;
			}

			if(moveForward(40)){
				ROS_WARN_STREAM("Set Waypoint Success!");
			}
			else {
				ROS_WARN_STREAM("Set Waypoint Failed!");
			}

			ROS_WARN_STREAM("Destination Set!");
			if(changeFlightMode("AUTO")){
				ROS_WARN_STREAM("Changed to AUTO!");
			}
			else {

				ROS_ERROR_STREAM("Failed changing to AUTO!");
			}


				node_feedback.data = "nc:navigation.end";
			  pub_node_select.publish(node_feedback);
				ros::shutdown();
			// if flightmode is HOLD, continue code
		}
		overrideRCControl(320, center_buoy_x, BASESPEED, 0);

	}
}

void nodeSelectCB(const std_msgs::String& msg){

	node_status.data = msg.data;
}


void imageProcessingCB(const auvsi16::ImageProcessing& msg){

	center_buoy_x	= msg.center_buoy_x;
	center_buoy_y = msg.center_buoy_y;
	buoy_area			= msg.buoy_area;
	radius_buoy		= msg.radius_buoy;
	buoy_number  	= msg.buoy_number;
	imgproc_status = msg.detection_status;
}
