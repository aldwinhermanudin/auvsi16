#include "../../include/auvsi16/basic_mission_function.hpp"
#include "../../include/auvsi16/auvsi16_general.hpp"

#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <auvsi16/ImageProcessing.h>
#include "mavros_msgs/State.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include "cmt/CMT.h"
#include "cmt/gui.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif

#define RATIO 3

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::waitKey;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;
using ::atof;

static string WIN_NAME = "Navigation";

int display					(Mat im, CMT & cmt);
void 	imageFrontReceiveCB	(const sensor_msgs::ImageConstPtr& msg);
void readImage				(Mat &output_image);
double rect_size			(RotatedRect rect);

bool 			from_file		= false;

// ###########################################################

#define	 MIN_GROUND_SPEED 0.2

void nodeSelectCB(const std_msgs::String& msg);
void imageProcessingCB				(const auvsi16::ImageProcessing& msg);
void stateCB									(const mavros_msgs::State& msg);
bool checkAUTOCruise					();
void checkGroundSpeed					();
bool changeFlightModeDebug		(string fm);
void vfrHUDCB									(const mavros_msgs::VFR_HUD& msg);
void startAUTOandChecker			();
ros::Publisher pub_imgproc_select;

double	ground_speed = 0;
int 		center_buoy_x	= 0;
int 	arena_distance	= 46;
int 	video_setpoint = 320;
double 	heading;
double first_lat;
double first_long;
double second_lat;
double second_long;
double last_center_buoy_x;

double initial_size = 0;
double current_size = 0;
bool sizeCheck();

std_msgs::String node_status;
std_msgs::String node_feedback;
string	state;
cv::Mat front_image;
int main(int argc, char **argv){

	ros::init(argc, argv, "navigation_mission_simple");
	ros::NodeHandle nh("~");

	nh.getParam("arena_distance", arena_distance);
	nh.getParam("heading", heading);
	nh.getParam("video_setpoint", video_setpoint);
	image_transport::ImageTransport it(nh);
	setBMFConfiguration(nh);

	RoverAUTOConfiguration auto_speed_conf;

	changePID(0.3, 0, 0);

	ros::Subscriber 						sub_state 			= nh.subscribe("/mavros/state", 1, stateCB);
	ros::Subscriber 						sub_vfr_hud 		= nh.subscribe("/mavros/vfr_hud", 1, vfrHUDCB);

	ros::Publisher pub_run_status	= nh.advertise<std_msgs::String>("/auvsi16/mission/navigation/status", 16);
	ros::Publisher pub_node_select	= nh.advertise<std_msgs::String>("/auvsi16/node/select", 16,true);
	ros::Subscriber sub_node_select = nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);
	image_transport::Subscriber sub_video_front = it.subscribe("/auvsi16/video/front", 1, imageFrontReceiveCB); // Topik subscribe: auvsi16/video
	
	ROS_WARN_STREAM("Waiting for navigation mission selected.");
	while (ros::ok() && node_status.data != "nm:navigation.start"){
		ros::spinOnce();
	}
	ROS_WARN_STREAM("Navigation mission selected.");

	while (ros::ok() && front_image.empty()){
		ros::spinOnce();
	}
	
	ROS_WARN_STREAM("Video Feed Received");
	
	//Create a CMT object
    CMT cmt;

    //Initialization bounding box
    Rect rect;

    //Parse args
    int display_video = 1;

    //Set up logging
    FILELog::ReportingLevel() = logINFO;

    //Create window
    namedWindow(WIN_NAME,CV_WINDOW_NORMAL);

    bool show_preview	= true;


    //Show preview until key is pressed
    while (show_preview && ros::ok())
    {
        Mat preview;
        readImage(preview);

        screenLog(preview, "Press a key to start selecting an object.");
        imshow(WIN_NAME, preview);

        char k = waitKey(10);
        if (k != -1) {
            show_preview = false;
        }
    }

    //Get initial image
    Mat im0;
    readImage(im0);

    rect = getRect(im0, WIN_NAME);
	initial_size = rect.width * rect.height;
    ROS_INFO_STREAM("Using " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height
        << " | size " << initial_size << " as initial bounding box.");
	
    //Convert im0 to grayscale
    Mat im0_gray;
    if (im0.channels() > 1) {
        cvtColor(im0, im0_gray, CV_BGR2GRAY);
    } else {
        im0_gray = im0;
    }

    //Initialize CMT
    cmt.initialize(im0_gray, rect);

    //Main loop
    while (true && ros::ok())
    {
        Mat im;
        readImage(im);

        if (im.empty()) break; //Exit at end of video stream

        Mat im_gray;
        if (im.channels() > 1) {
            cvtColor(im, im_gray, CV_BGR2GRAY);
        } else {
            im_gray = im;
        }

        //Let CMT process the frame
        cmt.processFrame(im_gray);
		
		center_buoy_x = cmt.bb_rot.center.x;
		
		current_size = rect_size(cmt.bb_rot);
		
		double compass_hdg_at_capture = compass_hdg;
		ROS_INFO_STREAM("Compass Heading : " << compass_hdg);
		ROS_INFO_STREAM("Compass Heading Current: " << compass_hdg_at_capture);
		ROS_INFO_STREAM("Latitude : " << global_position.latitude);
		ROS_INFO_STREAM("Longitude : " <<  global_position.longitude);
		ROS_INFO_STREAM("X Position : " << center_buoy_x);
		ROS_INFO_STREAM("Ratio : " << current_size/initial_size );
		//ROS_WARN_STREAM("Detected Red Buoy : " << red_buoy);
		//if (areaCheck(center_buoy_x, 320, 2)){
		if (sizeCheck()){

			compass_hdg = compass_hdg_at_capture;
			wp_sender->clearWaypointList();
			moveToHeading(arena_distance, heading);
			if(wp_sender->sendWaypointList()){
				ROS_WARN_STREAM("Set Waypoint Success!");
			}
			else {
				ROS_WARN_STREAM("Set Waypoint Failed!");
			}

			ROS_WARN_STREAM("Destination Set!");

			auto_speed_conf.setFullSpeed();
			if(auto_speed_conf.sendParameter()){
				ROS_INFO("Success change speed to Full power");
			}
			else{

					ROS_INFO("Failed change speed to Full power");
			}
			sleep(5);
			startAUTOandChecker();

			pub_node_select.publish(node_feedback);
			auto_speed_conf.setNormalSpeed();
			if(auto_speed_conf.sendParameter()){
				ROS_INFO("Success change speed to Normal power");
			}
			else{

					ROS_INFO("Failed change speed to Normal power");
			}
			sleep(5);
			node_feedback.data = "nc:navigation.end";
			ros::shutdown();
			// if flightmode is HOLD, continue code
		}
		
		if (cmt.points_active.size() > 0){
			overrideRCControl(video_setpoint, center_buoy_x, BASESPEED, 0);
		}
		else {
			center_buoy_x = last_center_buoy_x;
			overrideRCControl(video_setpoint, center_buoy_x, BASESPEED, 0);
		}
		
		last_center_buoy_x = center_buoy_x;
		
        ROS_INFO_STREAM("X Coordinates: " << cmt.bb_rot.center.x << " | Y Coordinates: " << cmt.bb_rot.center.x) ;

		char key = display(im, cmt);
		if(key == 'q') break;
		
    }
}
void nodeSelectCB(const std_msgs::String& msg){

	node_status.data = msg.data;
}

void stateCB(const mavros_msgs::State& msg){
	state = msg.mode;
}

void 	vfrHUDCB	(const mavros_msgs::VFR_HUD& msg){

	ground_speed = msg.groundspeed;
}

bool checkAUTOCruise(){

	if(state == "AUTO"){

		return true;
	}

	else{

		return false;
	}

}
void checkGroundSpeed(){
	if(ground_speed < MIN_GROUND_SPEED && checkAUTOCruise() ){
		changeFlightModeDebug("MANUAL");
		sleep(5);
		changeFlightModeDebug("AUTO");
		sleep(5);
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

void startAUTOandChecker(){
	changeFlightModeDebug("AUTO");
	sleep(1);
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	while(checkAUTOCruise()) {
		checkGroundSpeed();
		ros::spinOnce();
	}
}


int display(Mat im, CMT & cmt)
{
    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    for(size_t i = 0; i < cmt.points_active.size(); i++)
    {
        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
    }

    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
    }

    imshow(WIN_NAME, im);

    return waitKey(5);
}

double rect_size(RotatedRect rect){

    return (rect.size.width)*(rect.size.height);
}

void readImage(Mat &output_image){
		ros::spinOnce();
		if(!front_image.empty()){
			output_image = front_image.clone();
		}
}

void imageFrontReceiveCB(const sensor_msgs::ImageConstPtr& msg){

	try {
		front_image  = cv_bridge::toCvCopy(msg, "bgr8")->image;		// use cv_bridge::toCvCopy instead of cv_bridge::toCvShare to process a mat data
	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

bool sizeCheck(){
	
	if(current_size/initial_size >= RATIO){
		return true;
	}
	else {
		return false;
	}
}
