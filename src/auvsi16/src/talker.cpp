#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("auvsi16/belajar", 1000);

  int angkaMasukkan;
  while (ros::ok())
  {
    std_msgs::Int32 msg;
	   
    scanf("%d", &angkaMasukkan);
    msg.data = angkaMasukkan;

    ROS_INFO("%d", msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

  }

  return 0;
}

