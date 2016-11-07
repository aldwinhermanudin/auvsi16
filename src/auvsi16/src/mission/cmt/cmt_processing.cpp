#include "CMT.h"
#include "gui.h"
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

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
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

static string WIN_NAME = "tShroom";

int display(Mat im, CMT & cmt);
string write_rotated_rect(RotatedRect rect);
double rect_size(RotatedRect rect);


int main(int argc, char **argv)
{
    //Create a CMT object
    CMT cmt;

    //Initialization bounding box
    Rect rect;

    //Parse args
    int bbox_flag = 0;
    string input_path;
    string output_path;

    //input_path = "../tShroom_480p.mp4";
    
    //Set up logging
    FILELog::ReportingLevel() = logINFO;
    
    //Normal mode

    //Create window
    namedWindow(WIN_NAME);

    VideoCapture cap;

    bool show_preview = true;

    //If no input was specified
    if (input_path.length() == 0)
    {
        cap.open(0); //Open default camera device
    }

    //Else open the video specified by input_path
    else
    {
        cap.open(input_path);
        show_preview = false;
    }

    //If it doesn't work, stop
    if(!cap.isOpened())
    {
        cerr << "Unable to open video capture." << endl;
        return -1;
    }

    //Show preview until key is pressed
    while (show_preview)
    {
        Mat preview;
        cap >> preview;

        screenLog(preview, "Press a key to start selecting an object.");
        imshow(WIN_NAME, preview);

        char k = waitKey(10);
        if (k != -1) {
            show_preview = false;
        }
    }

    //Get initial image
    Mat im0;
    cap >> im0;

    //If no bounding was specified, get it from user
    if (!bbox_flag)
    {
        rect = getRect(im0, WIN_NAME);
    }

    FILE_LOG(logINFO) << "Using " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height
        << " as initial bounding box.";

    //Convert im0 to grayscale
    Mat im0_gray;
    if (im0.channels() > 1) {
        cvtColor(im0, im0_gray, CV_BGR2GRAY);
    } else {
        im0_gray = im0;
    }

    //Initialize CMT
    cmt.initialize(im0_gray, rect);

    int frame = 0;

    //Main loop
    while (true)
    {
        frame++;

        Mat im;
       
        cap >> im; //Else use next image in stream

        if (im.empty()) break; //Exit at end of video stream

        Mat im_gray;
        if (im.channels() > 1) {
            cvtColor(im, im_gray, CV_BGR2GRAY);
        } else {
            im_gray = im;
        }

        //Let CMT process the frame
        cmt.processFrame(im_gray);

        //TODO: Provide meaningful output
        FILE_LOG(logINFO) << "#" << frame << " active: " << cmt.points_active.size() << " | " << rect_size(cmt.bb_rot) ;

        //Display image and then quit if requested.
        char key = display(im, cmt);
        if(key == 'q') break;
    }

    return 0;
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

string write_rotated_rect(RotatedRect rect)
{
    Point2f verts[4];
    rect.points(verts);
    stringstream coords;

    coords << rect.center.x << "," << rect.center.y << ",";
    coords << rect.size.width << "," << rect.size.height << ",";
    coords << rect.angle << ",";

    for (int i = 0; i < 4; i++)
    {
        coords << verts[i].x << "," << verts[i].y;
        if (i != 3) coords << ",";
    }

    return coords.str();
}

double rect_size(RotatedRect rect){
   
    return (rect.size.width)*(rect.size.height);
}
