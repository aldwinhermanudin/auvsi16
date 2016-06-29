// ########################################## start of MavrosInterface class ########################################## //
#include "ros/ros.h"
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/ParamGet.h"
#include <iostream>
#include <string>

using namespace std;

class MavlinkParameter {

protected:
  ros::ServiceClient client_get_param;
  ros::ServiceClient client_set_param;
  ros::NodeHandle nh;

public:
  MavlinkParameter            ();
  bool setParameter         (string param_id, int param_value);
  bool setParameter         (string param_id, double param_value);
  double getDoubleParameter (string param_id);
  int getIntParameter       (string param_id);
};

MavlinkParameter::MavlinkParameter(){
  client_get_param = nh.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
	client_set_param = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
  }

bool MavlinkParameter::setParameter(string param_id, int param_value){

    mavros_msgs::ParamSet param_set;

    param_set.request.param_id = param_id;
    param_set.request.value.integer = param_value;
    param_set.request.value.real = 0.0;

    bool success_set_param = client_set_param.call(param_set);

    if (success_set_param){
      return true;
    } else {
      return false;
    }
}

bool MavlinkParameter::setParameter(string param_id, double param_value){

    mavros_msgs::ParamSet param_set;

    param_set.request.param_id = param_id;
    param_set.request.value.integer = 0;
    param_set.request.value.real = param_value;

    bool success_set_param = client_set_param.call(param_set);

    if (success_set_param){
      return true;
    } else {
      return false;
    }
}

double MavlinkParameter::getDoubleParameter (string param_id){
  mavros_msgs::ParamGet param_get;

  param_get.request.param_id = param_id;

  bool success_get_param = client_get_param.call(param_get);

  if (success_get_param){
    return param_get.response.value.real;
  } else {
    return 0;
  }
}

int MavlinkParameter::getIntParameter (string param_id){
  mavros_msgs::ParamGet param_get;

  param_get.request.param_id = param_id;

  bool success_get_param = client_get_param.call(param_get);

  if (success_get_param){
    return param_get.response.value.integer;
  } else {
    return 0;
  }
}

// ########################################## start of MavrosInterface class ########################################## //

// ########################################## start of MavrosInterface class ########################################## //
class RoverAUTOConfiguration : public MavlinkParameter {
private:
  double steer2srv_p   = 1.5;
  double turn_max_g    = 1;
  int navl1_period     = 14;
  int speed_turn_gain  = 100;
  double cruise_speed  = 1;
  int cruise_throttle  = 9;

public:
  RoverAUTOConfiguration            ();
  void setAUTOParameterValue    (double steer2srv_p, double turn_max_g, int navl1_period, int speed_turn_gain, double cruise_speed, int cruise_throttle);
  bool sendParameter        ();
  void setFullSpeed         ();
  void setNormalSpeed       ();
};

RoverAUTOConfiguration::RoverAUTOConfiguration()
: MavlinkParameter(){}

void RoverAUTOConfiguration::setAUTOParameterValue    (double steer2srv_p, double turn_max_g, int navl1_period, int speed_turn_gain, double cruise_speed, int cruise_throttle){
  this->steer2srv_p   = steer2srv_p;
  this->turn_max_g    = turn_max_g;
  this->navl1_period     = navl1_period;
  this->speed_turn_gain  = speed_turn_gain;
  this->cruise_speed  = cruise_speed;
  this->cruise_throttle  = cruise_throttle;
}

void RoverAUTOConfiguration::setFullSpeed(){
  this->setAUTOParameterValue(1.5, 1, 14, 100, 2, 43);
}
void RoverAUTOConfiguration::setNormalSpeed(){
  this->setAUTOParameterValue(1.5, 1, 14, 100, 1, 9);
}

bool RoverAUTOConfiguration::sendParameter (){

  bool param1 = this->setParameter("STEER2SRV_P", steer2srv_p);
  bool param2 = this->setParameter("TURN_MAX_G", turn_max_g);
  bool param3 = this->setParameter("NAVL1_PERIOD", navl1_period);
  bool param4 = this->setParameter("SPEED_TURN_GAIN", speed_turn_gain);
  bool param5 = this->setParameter("CRUISE_SPEED", cruise_speed);
  bool param6 = this->setParameter("CRUISE_THROTTLE", cruise_throttle);

  if(param1 && param2 && param3 && param4 && param5 && param6){
    return true;
  }
  else{
    return false;
  }
}

// ########################################## start of MavrosInterface class ########################################## //
