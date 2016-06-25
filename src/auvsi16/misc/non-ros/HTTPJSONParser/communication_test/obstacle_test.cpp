#include "../include/rapidjson/document.h"

#include "../include/rapidjson/writer.h"
#include "../include/rapidjson/stringbuffer.h"

#include "../include/auvsicommunication.hpp"
#include <iostream>

using namespace rapidjson;

int main() {

  string	challenge_status	= "idle";
  string	server_ip					=	"192.168.1.11";
  int 		server_port				=	80;
  string 	team_code					=	"rooster.php";
  string	course_type 			= "courseA";

  getTime time_lord;
  ObstacleMessage auvsi_protocol(server_ip, server_port, course_type, team_code);

  auvsi_protocol.setPayloadCommunication();
  cout << auvsi_protocol.getPayload() << endl << endl;
  auvsi_protocol.sendTCP();
  cout << auvsi_protocol.getResponse() << endl << endl;
  cout << auvsi_protocol.decodeResponeStatus() << endl;

  return 0;
}
