#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

class VideoResizer
{

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
	double resize_width_factor = 0.5;
	double resize_height_factor = 0.5;

public:

  VideoResizer(string subscriber_topic, string publisher_topic,double resize_width_factor,double resize_height_factor)
    : it_(nh_){
    // Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe(subscriber_topic, 1, &VideoResizer::imageCb, this);
    image_pub_ = it_.advertise(publisher_topic, 1);
		this->resize_width_factor = resize_width_factor;
		this->resize_height_factor = resize_height_factor;

	}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

		resize(cv_ptr->image, cv_ptr->image, Size(), resize_width_factor, resize_height_factor, INTER_LINEAR);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_feed_upstream");
	ros::NodeHandle nh_("~");
	double resize_width_factor	= 0.5;
	double resize_height_factor	= 0.5;
	nh_.getParam("resize_width_factor", resize_width_factor);
	nh_.getParam("resize_height_factor", resize_height_factor);
  VideoResizer front_video("/auvsi16/video/front", "/auvsi16/video/front/resize", resize_width_factor, resize_height_factor);
  VideoResizer right_video("/auvsi16/video/right", "/auvsi16/video/right/resize", resize_width_factor, resize_height_factor);
  ros::spin();
  return 0;
}
