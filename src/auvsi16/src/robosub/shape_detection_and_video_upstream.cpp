#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/CompressedImage.h>
#include <vector>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

vector<int> param = vector<int>(2);     

RNG rng(12345);

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

void imageProcessing(Mat imgInput, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius);

int iLowH 	= 0;	// yellow = 15, red = 166 , blue = 92
int iHighH 	= 179;	// yellow = 29, red = 179 , blue = 114
	
int iLowS	= 90; // lower this to allow more noise, 80 is normal
int iHighS 	= 255;
	
int iLowV 	= 0; // lower this to allow more noise
int iHighV 	= 255;

cv::Mat dst;	

int main( int argc, char** argv ){
	
	ros::init(argc, argv, "robosub_initial"); // Nama program: video_listener
	ros::NodeHandle nh("~");
	  
	param[0] = CV_IMWRITE_JPEG_QUALITY;			// set tipe encoding 
	param[1] = 30;								// set kualitas encoding
	  
	int video_port = 0;
	int capture_type = 0;
	nh.getParam("jpeg_quality", param[1]);
	nh.getParam("video_port", video_port);
	nh.getParam("capture_type", capture_type);
		
	image_transport::ImageTransport it(nh);
	
    ros::Publisher pub_video_front = nh.advertise<sensor_msgs::CompressedImage>("/auvsi16/robosub/video/front", 1, true);
  
	
	VideoCapture cap(video_port); //capture the video from webcam
	
	if (!cap.isOpened()){  // if not success, exit program
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	
	while (ros::ok()){
		
		Mat imageRead;//	 = imread(argv[1], CV_LOAD_IMAGE_UNCHANGED);
	
		bool bSuccess = cap.read(imageRead); // read a new frame from video
		if (!bSuccess){ //if not success, break loop
			
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}
		
		vector<uchar> buff;
		
		if (!capture_type) {
			int center_buoy_x	= 0;
			int center_buoy_y 	= 0;
			double buoy_area	= 0;
			double radius_buoy	= 0;
			
			imageProcessing(imageRead,500, &center_buoy_x, &center_buoy_y, &buoy_area, &radius_buoy);
			
			imencode(".jpeg",dst, buff,param);
		}
		else {
			
			imencode(".jpeg",imageRead, buff,param);
		}
		
		sensor_msgs::CompressedImage buff_message;
		buff_message.data = buff;
		pub_video_front.publish(buff_message);
	}
	
	return 0;
}

void imageProcessing(Mat imgInput, int minimum_area, int *detected_center_x, int *detected_center_y, double *contour_area, double* detected_radius){
	
	Mat imgOriginal = imgInput.clone();
		
	GaussianBlur( imgOriginal, imgOriginal, Size( 5, 5 ), 0, 0 );
	
	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
	
	Mat imgThresholded;
	
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	
	cv::Mat bw;
	cv::Canny(imgThresholded, bw, 0, 50, 5);
	
	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;
	dst = imgInput.clone();
	
	
	for (int i = 0; i < contours.size(); i++)
	{
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if (std::fabs(cv::contourArea(contours[i])) < minimum_area)
			continue;

		if (approx.size() == 3 &&  cv::isContourConvex(approx))
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
						
			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			
			for(int z = 0; z < cos.size(); z++){
				cout << "Triangle : " << cos[z] << endl;
			}
			
			if (mincos >= 0.38 && maxcos <= 0.62){
				setLabel(dst, "SEGITIGA", contours[i]);    // Triangles
			}
			
		
		}
		
		else if (approx.size() == 12 &&  !cv::isContourConvex(approx))
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
						
			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			
			for(int z = 0; z < cos.size(); z++){
				cout << "CROSS : " << cos[z] << endl;
			}
									
			if (mincos >= -0.5 && maxcos <= 0.5){
				setLabel(dst, "PALANG", contours[i]);    // Triangles
			}
			
		}
	}
}
