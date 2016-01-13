#include "ros/ros.h"
#include "std_msgs/Int32.h"

int input;

void chatterCallback(const std_msgs::Int32& msg)
{
	ROS_INFO("I heard: [%d]", msg.data);
	input = msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("auvsi16/belajar", 1000, chatterCallback);
  
  while (ros::ok()) {
	  int output;
	  int last_output;

	  output = input*input*input*input*input;
	  
	  if (output != last_output) {
	  printf("Hasil %d \n", output);
		}
		
	last_output = output;
  
	  ros::spinOnce();
	}

  return 0;
}
