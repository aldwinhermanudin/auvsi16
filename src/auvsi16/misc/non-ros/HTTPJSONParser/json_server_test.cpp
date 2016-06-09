#include "include/rapidjson/document.h"

#include "include/rapidjson/writer.h"
#include "include/rapidjson/stringbuffer.h"

#include "include/auvsicommunication.hpp"
#include <iostream>

using namespace rapidjson;

int main() {

  string	challenge_status	= "idle";
  string	server_ip					=	"127.0.0.1";
  int 		server_port				=	80;
  string 	team_code					=	"rooster.php";
  string	course_type 			= "courseA";

  getTime time_lord;
  HeartbeatMessage auvsi_protocol(server_ip, server_port, course_type, team_code);

  auvsi_protocol.setPayloadCommunication(time_lord.getYMDHS(), challenge_status, 102.231,-32.231);
  //auvsi_protocol.setPayload("{\"timestamp\":\"20160609102641\",\"challenge\":\"idle\",\"position\":{\"datum\":\"WGS84\",\"latitude\":102.231003,\"longitude\":-32.230999}}")
  auvsi_protocol.sendTCP();
  cout << auvsi_protocol.getPayload() << endl << endl;
  cout << auvsi_protocol.getRespone() << endl << endl;

  return 0;
}
