#include "../include/auvsi16/basic_mission_function.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

void 	gpsVelocityCB							(const geometry_msgs::TwistStamped& msg);
void 	dockingGPSCB							(const mavros_msgs::Waypoint& msg);
void 	sonarDataCB								(const auvsi16::sonarData& msg);
void 	imageFrontReceiveCB				(const sensor_msgs::ImageConstPtr& msg);
void 	imageRightReceiveCB				(const sensor_msgs::ImageConstPtr& msg);
static double angle							(cv::Point pt1, cv::Point pt2, cv::Point pt0);
int 	imageProcessing						(int shape_type, Mat imgInput, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius, Mat& canny_detection);
void 	dockingMissionExec				();
void 	setHSVRange								(int lowh, int highh, int lows, int highs, int lowv, int highv);
void 	setDockingWaypoint				(double x_dock_gps, double y_dock_gps, double distance_from_triangle, double distance_from_zero);
void 	initializeDockingMission	(double distance_from_triangle, double distance_from_zero);
void 	imageProcessingDebug			();
void 	imageProcessingDisplay		(Mat canny_detect);
void 	resetDockingPoint					();
void nodeSelectCB								(const std_msgs::String& msg);

int			iLowH 		= 0;	// yellow = 15, red = 166 , blue = 92
int 		iHighH 		= 179;	// yellow = 29, red = 179 , blue = 114
int 		iLowS			= 90; // lower this to allow more noise, 80 is normal
int 		iHighS 		= 255;
int 		iLowV 		= 0; // lower this to allow more noise
int 		iHighV 		= 255;

double 	gps_vel_x = 0;
double 	gps_vel_y = 0;
double 	gps_vel_z = 0;

int 		sonar_data[13];

bool										dock_gps_status = false;
double 									docking_coordinates_latitude[3];
double									docking_coordinates_longitude[3];
mavros_msgs::Waypoint 	center_dock_gps;
std_msgs::String node_status;
std_msgs::String node_feedback;

double shape_coordinates_latitude[3];
double shape_coordinates_longitude[3];

double heading_zero 		= 0;
double heading_triangle = 0;
double heading_one 			= 0;
double heading_two 			= 0;

cv::Mat front_image;
cv::Mat right_image;

int 		center_buoy_x	= 0;
int 		center_buoy_y = 0;
double	buoy_area			= 0;
double 	radius_buoy		= 0;
int 		buoy_number  	= 0;

int main(int argc, char **argv){

	ros::init(argc, argv, "docking_mission");
	ros::NodeHandle nh("~");
	ROS_INFO("Docking Mission Start!");

	image_transport::ImageTransport it(nh);
	setBMFConfiguration(nh);
	nh.getParam("heading_zero", heading_zero);

	image_transport::Subscriber sub_video_front = it.subscribe("/auvsi16/video/front", 1, imageFrontReceiveCB); // Topik subscribe: auvsi16/video
	image_transport::Subscriber sub_video_right = it.subscribe("/auvsi16/video/right", 1, imageRightReceiveCB); // Topik subscribe: auvsi16/video
	ros::Subscriber 						sub_sonar_data 	= nh.subscribe("/auvsi16/sonar_data", 1, sonarDataCB);
	ros::Subscriber 						sub_gps_vel 		= nh.subscribe("/mavros/global_position/raw/gps_vel", 1, gpsVelocityCB);
	ros::Subscriber 						sub_docking_gps = nh.subscribe("/auvsi16/mission/docking", 1, dockingGPSCB);

	ros::Publisher pub_run_status		= nh.advertise<std_msgs::String>("/auvsi16/mission/docking/status", 16);
	ros::Publisher pub_node_select = nh.advertise<std_msgs::String>("/auvsi16/node/select", 16,true);
	ros::Subscriber sub_node_select 			= nh.subscribe("/auvsi16/node/select", 10, nodeSelectCB);


	namedWindow("Image Canny", CV_WINDOW_NORMAL); //create a window called "Thresholded Image"
	namedWindow("Image", CV_WINDOW_NORMAL); //create a window called "Thresholded Image"

	int distance_from_shape = 7;
	int distance_to_dock 		= 2;
	int distance_to_away		= 10;
	int detection_accuracy  = 15;

	ROS_WARN_STREAM("Waiting for docking mission selected.");
	while (ros::ok() && node_status.data.compare("dm:docking.start") != 0){
				ros::spinOnce();
	}

	initializeDockingMission(distance_from_shape,20);

	bool first_cross_status		= false;
	bool triangle_status			= false;
	bool second_cross_status	= false;

	wp_sender->clearWaypointList();	// clear waypoint list

	while(ros::ok() && !first_cross_status){

		ros::spinOnce();
		Mat canny_detection;

		buoy_number = imageProcessing(12, right_image, 0, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy, canny_detection);
		imageProcessingDebug();
		imageProcessingDisplay(canny_detection);
		if (areaCheck(center_buoy_x, 640, detection_accuracy)){
			
			{
				long double x_target;
				long double y_target;
				double target_latitude;
				double target_longitude;

				positionEstimation(distance_to_away,heading_zero,&x_target, &y_target);
				calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);

				wp_sender->addWaypoint(target_latitude, target_longitude);	// add waypoint to list
			}

			{
				long double x_target;
				long double y_target;
				double target_latitude;
				double target_longitude;

				positionEstimation(distance_to_dock,heading_triangle,&x_target, &y_target);
				calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);

				wp_sender->addWaypoint(target_latitude, target_longitude);	// add waypoint to list
			}

			first_cross_status = true;
		}
	}
	ROS_WARN_STREAM("First Cross Set");

	resetDockingPoint();

	while(ros::ok() && !triangle_status){

		ros::spinOnce();
		Mat canny_detection;

		buoy_number = imageProcessing(3, right_image, 0, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy, canny_detection);
		imageProcessingDebug();
		imageProcessingDisplay(canny_detection);
		if (areaCheck(center_buoy_x, 640, detection_accuracy)){


			{
				long double x_target;
				long double y_target;
				double target_latitude;
				double target_longitude;

				positionEstimation(distance_to_away,heading_zero,&x_target, &y_target);
				calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);

				wp_sender->addWaypoint(target_latitude, target_longitude);	// add waypoint to list
			}

			{
				long double x_target;
				long double y_target;
				double target_latitude;
				double target_longitude;

				positionEstimation(distance_to_dock,heading_triangle,&x_target, &y_target);
				calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);

				wp_sender->addWaypoint(target_latitude, target_longitude);	// add waypoint to list
			}

			triangle_status = true;
		}
	}

	ROS_WARN_STREAM("Triangle Set");

	resetDockingPoint();

	while(ros::ok() && !second_cross_status){

		ros::spinOnce();
		Mat canny_detection;

		buoy_number = imageProcessing(12, right_image, 0, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy, canny_detection);
		imageProcessingDebug();
		imageProcessingDisplay(canny_detection);
		if (areaCheck(center_buoy_x, 640, detection_accuracy)){


			{
				long double x_target;
				long double y_target;
				double target_latitude;
				double target_longitude;

				positionEstimation(distance_to_away,heading_zero,&x_target, &y_target);
				calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);

				wp_sender->addWaypoint(target_latitude, target_longitude);	// add waypoint to list
			}

			{
				long double x_target;
				long double y_target;
				double target_latitude;
				double target_longitude;

				positionEstimation(distance_to_dock,heading_triangle,&x_target, &y_target);
				calculateCoordinate(global_position.latitude, global_position.longitude, global_position.altitude, &target_latitude,&target_longitude,x_target,y_target);

				wp_sender->addWaypoint(target_latitude, target_longitude);	// add waypoint to list
			}

			second_cross_status = true;
		}
	}
	ROS_WARN_STREAM("Second Cross Set");

	if(changeFlightMode("MANUAL")){
		ROS_WARN_STREAM("Changed to MANUAL!");
	}
	else {

		ROS_ERROR_STREAM("Failed changing to MANUAL!");
	}

	sleep(5);

	bool success_set = wp_sender->sendWaypointList();	// send waypoint to fcu
	// Check for success and use the response .
	if(success_set){
		ROS_INFO_STREAM("Set Waypoint Success");
	}
	else {
		ROS_INFO_STREAM("Set Waypoint Failed");
	}

	node_feedback.data = "nc:docking.end";
  pub_node_select.publish(node_feedback);
	ros::shutdown();
}

void dockingMissionExec(){

	Mat canny_detection;
	std::vector<cv::Point> contour_shape;

	buoy_number = imageProcessing(12, right_image, 500, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy, canny_detection);
	//imageProcessingDebug(canny_detection);
}

void imageProcessingDebug(){

	cout << "######## Posisi Shape #########" << endl;
	cout << "Detected Shape : " << buoy_number << endl;
	cout << "Posisi X  : " << center_buoy_x << endl;
	cout << "Posisi Y  : " << center_buoy_y << endl;
	cout << "Shape Size : " << buoy_area << endl;
	cout << endl;
	cout << endl;

}

void imageProcessingDisplay(Mat canny_detect){

	RNG rng(12345);
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	Point2f circle_center;
	circle_center.x = center_buoy_x;
	circle_center.y = center_buoy_y;
	circle( right_image, circle_center, radius_buoy, color, 4, 8, 0 );

	imshow("Image Canny", canny_detect );
	imshow("Image", right_image );
	waitKey(30);
}

int imageProcessing(int shape_type, Mat imgInput, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius, Mat& canny_detection){

	Mat imgOriginal = imgInput.clone();

	GaussianBlur( imgOriginal, imgOriginal, Size( 5, 5 ), 0, 0 );

	Mat imgHSV;
	Mat imgThresholded;

	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	cv::Mat bw;
	cv::Canny(imgThresholded, bw, 0, 50, 5);

	// canny mat frame, variable passing
	canny_detection = bw.clone();

	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;

	Point2f center;
	float radius;
	int largest_shape_area = 0;
	int largest_shape_id = 0;
	int number_of_detected_shape = 0;
	for (int i = 0; i < contours.size(); i++){
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

		if (shape_type == 3 && approx.size() == 3 && std::fabs(cv::contourArea(contours[i])) > minimum_area && cv::isContourConvex(approx))
		{
			// Number of vertices of polygonal curve
			int vtc = approx.size();

			// Get the cosines of all corners
			std::vector<double> cos;
			for (int j = 2; j < vtc+1; j++)
			cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

			// Sort ascending the cosine values
			std::sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			cout << "\n\n#### Triangle Angle ####" << endl;

			for(int z = 0; z < cos.size(); z++){
				cout << "Triangle : " << cos[z] << endl;
			}

			cout << "#### Triangle Angle ####\n\n" << endl;

			if (mincos >= 0.2 && maxcos <= 0.7){

				double a = contourArea( contours[i],false);  //  Find the area of contour
				if(a > largest_shape_area && a > minimum_area){
					largest_shape_area = a;
					largest_shape_id = i;                //Store the index of largest contour
					minEnclosingCircle( contours[i], center, radius);
				}
				if(largest_shape_area > minimum_area){
					number_of_detected_shape++;
				}
			}
		}

		else if (shape_type == 12 && approx.size() == 12 && !cv::isContourConvex(approx) && std::fabs(cv::contourArea(contours[i])) > minimum_area)
		{
			// Number of vertices of polygonal curve
			int vtc = approx.size();

			// Get the cosines of all corners
			std::vector<double> cos;
			for (int j = 2; j < vtc+1; j++)
			cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

			// Sort ascending the cosine values
			std::sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			cout << "\n\n#### CROSS Angle ####" << endl;
			for(int z = 0; z < cos.size(); z++){
				cout << "CROSS : " << cos[z] << endl;
			}
			cout << "#### CROSS Angle ####\n\n" << endl;

			if (mincos >= -0.5 && maxcos <= 0.5){

				double a = contourArea( contours[i],false);  //  Find the area of contour
				if(a > largest_shape_area && a > minimum_area){
					largest_shape_area = a;
					largest_shape_id = i;                //Store the index of largest contour
					minEnclosingCircle( contours[i], center, radius);
				}
				if(largest_shape_area > minimum_area){
					number_of_detected_shape++;
				}
			}

		}

	}

	if(largest_shape_area > minimum_area){

		*detected_center_x	=	(int)center.x;
		*detected_center_y	=	(int)center.y;
		*contour_area		=	largest_shape_area;
		*detected_radius	=	radius;
	}

	return number_of_detected_shape;
}

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0){
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void initializeDockingMission(double distance_from_triangle, double distance_from_zero){

	ROS_INFO("Waiting for Video Feed Mission Start!");
	// this it to whether image_received is empty or not, move this to a function.
	while (ros::ok() && front_image.empty() && right_image.empty()){
		ros::spinOnce();
	}
	ROS_INFO("Video Feed Received!");

	ROS_INFO_STREAM( "Waiting for docking location : " << dock_gps_status);
	while (ros::ok() && !dock_gps_status){
		ros::spinOnce();
	}
	ROS_INFO_STREAM( "Got Docking location\t\t: " << dock_gps_status);
	ROS_INFO_STREAM("Heading Towards Zero Point\t: " << heading_zero);
	ROS_INFO_STREAM("Docking Latitude\t\t\t: " << center_dock_gps.x_lat);
	ROS_INFO_STREAM("Docking Longitude\t\t\t: " << center_dock_gps.y_long);

	changePID(0.3, 0, 0);
	setHSVRange(0,179,90,255,0,255);
	setDockingWaypoint(center_dock_gps.x_lat, center_dock_gps.y_long, distance_from_triangle, distance_from_zero);
}

void setDockingWaypoint(double x_dock_gps, double y_dock_gps, double distance_from_triangle, double distance_from_zero){

	/*						*(1)
	*
	* 	   C#
	*
	*
	* 	   T# 		*(0)
	*
	*
	* 	   C#
	*
	*							*(2)
	*
	* heading towards (0) from T#
	*/

	heading_triangle = heading_zero + 180;
	if (heading_triangle > 360) heading_triangle = heading_triangle - 360;

	heading_one = heading_zero - 90;
	if (heading_one < 0) heading_one = 360 + heading_one;

	heading_two = heading_zero + 90;
	if (heading_two > 360) heading_two = heading_two - 360;

	long double x_target;
	long double y_target;;

	positionEstimation(distance_from_triangle,heading_zero,&x_target, &y_target);
	calculateCoordinate(x_dock_gps, y_dock_gps, 0, &docking_coordinates_latitude[0],&docking_coordinates_longitude[0],x_target,y_target);

	positionEstimation(distance_from_zero,heading_one,&x_target, &y_target);
	calculateCoordinate(docking_coordinates_latitude[0], docking_coordinates_longitude[0], 0, &docking_coordinates_latitude[1],&docking_coordinates_longitude[1],x_target,y_target);

	positionEstimation(distance_from_zero,heading_two,&x_target, &y_target);
	calculateCoordinate(docking_coordinates_latitude[0], docking_coordinates_longitude[0], 0, &docking_coordinates_latitude[2],&docking_coordinates_longitude[2],x_target,y_target);

	wp_sender->clearWaypointList();
	wp_sender->addWaypoint(docking_coordinates_latitude[1], docking_coordinates_longitude[1]);
	wp_sender->addWaypoint(docking_coordinates_latitude[0], docking_coordinates_longitude[0]);
	wp_sender->addWaypoint(docking_coordinates_latitude[2], docking_coordinates_longitude[2]);
	wp_sender->sendWaypointList();
}

void sonarDataCB(const auvsi16::sonarData& sonar_recv){

	for(int i=0; i < 13; i++) sonar_data[i] = sonar_recv.data[i];
}

void dockingGPSCB(const mavros_msgs::Waypoint& msg){

	if (!dock_gps_status){
		center_dock_gps = msg;
		dock_gps_status = true;
	}

}

void gpsVelocityCB(const geometry_msgs::TwistStamped& msg){

	gps_vel_x = msg.twist.linear.x;
	gps_vel_y = msg.twist.linear.y;
	gps_vel_z = msg.twist.linear.z;

}

void imageFrontReceiveCB(const sensor_msgs::ImageConstPtr& msg){

	try {
		front_image  = cv_bridge::toCvCopy(msg, "bgr8")->image;		// use cv_bridge::toCvCopy instead of cv_bridge::toCvShare to process a mat data
	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void imageRightReceiveCB(const sensor_msgs::ImageConstPtr& msg){

	try {
		right_image  = cv_bridge::toCvCopy(msg, "bgr8")->image;		// use cv_bridge::toCvCopy instead of cv_bridge::toCvShare to process a mat data
	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void setHSVRange(int lowh, int highh, int lows, int highs, int lowv, int highv){

	iLowH 	= lowh;	// yellow = 15, red = 166 , blue = 92
	iHighH 	= highh;	// yellow = 29, red = 179 , blue = 114

	iLowS		= lows; // lower this to allow more noise, 80 is normal
	iHighS 	= highs;

	iLowV 	= lowv; // lower this to allow more noise
	iHighV 	= highv;
}

void resetDockingPoint(){

	center_buoy_x	= 0;
	center_buoy_y = 0;
	buoy_area			= 0;
	radius_buoy		= 0;
	buoy_number  	= 0;
}


void nodeSelectCB(const std_msgs::String& msg){

	node_status.data = msg.data;
}
