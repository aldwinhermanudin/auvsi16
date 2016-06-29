#include "../../../include/auvsi16/interface_mavros.hpp"

using namespace std;
int main(int argc, char **argv)
{

  ros::init(argc, argv, "mission_clear");
  ros::NodeHandle n;

	RoverAUTOConfiguration set_param;
	set_param.setNormalSpeed();
	cout << set_param.sendParameter() << endl;
  return 0;
}
