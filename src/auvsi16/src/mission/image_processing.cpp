#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "auvsi16/HSVValue.h"
#include "auvsi16/ImageProcessing.h"

using namespace std;
using namespace cv;

void 	imageFrontReceiveCB			(const sensor_msgs::ImageConstPtr& msg);
void 	imageRightReceiveCB			(const sensor_msgs::ImageConstPtr& msg);
static double angle						(cv::Point pt1, cv::Point pt2, cv::Point pt0);
int 	dockingImageProcessing	(int shape_type, Mat imgInput, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius, Mat& canny_detection);
int   navigationImageProcessing	(Mat imgInput, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius, Mat& image_threshold);
void 	setHSVRange							(int lowh, int highh, int lows, int highs, int lowv, int highv);
void imageProcessingDisplayWindow();
void imageProcessingDebug();
void imageProcessingDisplay(Mat original_image, Mat image_debug);
void resetImageProcessingData();

void imageProcessingSelectCB(const std_msgs::Int16& msg);
void imageProcessingHSVCB(const auvsi16::HSVValue& msg);

cv::Mat right_image;
cv::Mat front_image;

int			iLowH 		= 0;	// yellow = 15, red = 166 , blue = 92
int 		iHighH 		= 179;	// yellow = 29, red = 179 , blue = 114
int 		iLowS			= 90; // lower this to allow more noise, 80 is normal
int 		iHighS 		= 255;
int 		iLowV 		= 0; // lower this to allow more noise
int 		iHighV 		= 255;

int 		center_buoy_x	= 0;
int 		center_buoy_y = 0;
double	buoy_area			= 0;
double 	radius_buoy		= 0;
int 		buoy_number  	= 0;
int 		image_processing_select = 0;

int main(int argc, char **argv){

	ros::init(argc, argv, "image_processing");
	ros::NodeHandle nh("~");
	ROS_INFO("Image Processing Start!");

	image_transport::ImageTransport it(nh);

	image_transport::Subscriber sub_video_front = it.subscribe("/auvsi16/video/front", 1, imageFrontReceiveCB); // Topik subscribe: auvsi16/video
	image_transport::Subscriber sub_video_right = it.subscribe("/auvsi16/video/right", 1, imageRightReceiveCB); // Topik subscribe: auvsi16/video

	ros::Publisher pub_imgproc_data		= nh.advertise<auvsi16::ImageProcessing>("/auvsi16/node/image_processing/data", 16);
	ros::Subscriber sub_imgproc_select	= nh.subscribe("/auvsi16/node/image_processing/select", 10, imageProcessingSelectCB);
	ros::Subscriber sub_imgproc_hsv	= nh.subscribe("/auvsi16/node/image_processing/hsv", 10, imageProcessingHSVCB);

	while (ros::ok() && front_image.empty() && right_image.empty()){
	//while (ros::ok() && front_image.empty()){
		ros::spinOnce();
	}
	imageProcessingDisplayWindow();
	auvsi16::ImageProcessing imgproc_data;

	while(ros::ok()){
		Mat image_debug;
		ros::spinOnce();
		if(image_processing_select == 0){
			resetImageProcessingData();
			imgproc_data.detection_status = false;
		}
		else if(image_processing_select == 1){
			buoy_number = dockingImageProcessing(3, right_image, 0, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy, image_debug);
			imgproc_data.detection_status = true;
			imageProcessingDebug();
			imageProcessingDisplay(right_image,image_debug);
		}
		else if(image_processing_select == 2){
			buoy_number = dockingImageProcessing(12, right_image, 0, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy, image_debug);
			imgproc_data.detection_status = true;
			imageProcessingDebug();
			imageProcessingDisplay(right_image,image_debug);
		}
		else if(image_processing_select == 3){
			buoy_number = navigationImageProcessing(front_image, 0, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy, image_debug);
			//buoy_number = navigationImageProcessing(right_image, 0, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy, image_debug);
			imgproc_data.detection_status = true;
			imageProcessingDebug();
		//	imageProcessingDisplay(right_image,image_debug);
			imageProcessingDisplay(front_image,image_debug);
		}
		else {
			resetImageProcessingData();
			imgproc_data.detection_status = false;
		}

		imgproc_data.buoy_number = buoy_number;
		imgproc_data.center_buoy_x = center_buoy_x;
		imgproc_data.center_buoy_y = center_buoy_y;
		imgproc_data.buoy_area = buoy_area;
		imgproc_data.radius_buoy = radius_buoy;
		pub_imgproc_data.publish(imgproc_data);
	}
}

void imageProcessingDisplayWindow(){

	namedWindow("Image Debug", CV_WINDOW_NORMAL); //create a window called "Thresholded Image"
	namedWindow("Image Original", CV_WINDOW_NORMAL); //create a window called "Thresholded Image"
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

void imageProcessingDisplay(Mat original_image, Mat image_debug){

	RNG rng(12345);
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	Point2f circle_center;
	circle_center.x = center_buoy_x;
	circle_center.y = center_buoy_y;
	circle( original_image, circle_center, radius_buoy, color, 4, 8, 0 );

	imshow("Image Debug", image_debug );
	imshow("Image Original", original_image );
	waitKey(30);
}

int navigationImageProcessing	(Mat imgInput, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius, Mat& image_threshold){

	Mat imgOriginal = imgInput.clone();

	GaussianBlur( imgOriginal, imgOriginal, Size( 5, 5 ), 0, 0 );

	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	Mat imgThresholded;

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	//morphological closing (removes small holes from the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	image_threshold = imgThresholded.clone();

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Point2f center;
	float radius;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

	int largest_bouy_area = 0;
	int largest_bouy_id = 0;
	int number_of_detected_buoy = 0;
	for( int i = 0; i< contours.size(); i++ ){
		double a = contourArea( contours[i],false);  //  Find the area of contour
		if(a > largest_bouy_area && a > minimum_area){
			largest_bouy_area = a;
			largest_bouy_id = i;                //Store the index of largest contour
			minEnclosingCircle( contours[i], center, radius);
		}
		if(largest_bouy_area > minimum_area){
			number_of_detected_buoy++;
		}
	}

	if(largest_bouy_area > minimum_area){
		*detected_center_x	=	(int)center.x;
		*detected_center_y	=	(int)center.y;
		*contour_area		=	largest_bouy_area;
		*detected_radius	=	radius;
	}

	return number_of_detected_buoy;
}

int dockingImageProcessing(int shape_type, Mat imgInput, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius, Mat& canny_detection){

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

void imageProcessingSelectCB(const std_msgs::Int16& msg){

	image_processing_select = msg.data;
}

void imageProcessingHSVCB(const auvsi16::HSVValue& msg){

	setHSVRange(msg.iLowH, msg.iHighH, msg.iLowS, msg.iHighS, msg.iLowV, msg.iHighV);
}

void resetImageProcessingData(){

	center_buoy_x	= 0;
	center_buoy_y = 0;
	buoy_area			= 0;
	radius_buoy		= 0;
	buoy_number  	= 0;
}
