#include <ros/ros.h>

int main(int argc, char* argv[])
{
    int serial_number = -1;
    ros::init(argc, argv, "parameter_test");

    //n is in the global namespace
    ros::NodeHandle n("~");
    //n.getParam("serial", serial_number);
    
	n.param("serial", serial_number, 42);
	
    ros::Rate loop_rate(10);
    ROS_INFO("Serial was %d", serial_number);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 * example of a lunch file to implement parameter

<launch>

  <node name="param_test" pkg="auvsi16" type="param_test" output="screen">
    <param name="serial" value="123" />
  </node>

</launch>

**/

