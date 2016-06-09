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
  string 	team_code					=	"AMVUI";
  string	course_type 			= "courseA";

  getTime time_lord;
  auvsiCommunication auvsi_protocol(server_ip, server_port, course_type, team_code,  communicationType::heartbeat);

  auvsi_protocol.setPayloadCommunication(time_lord.getYMDHS(), challenge_status, 102.231,-32.231);
  cout << auvsi_protocol.getPayload();

  // 1. Parse a JSON string into DOM.
  const char* json = "{\"project\":\"rapidjson\",\"stars\":10.12}";
  Document d;
  d.Parse(json);

  // 2. Modify it by DOM.
  Value& s = d["stars"];
  s.SetFloat(s.GetFloat() + 1);

  // 3. Stringify the DOM
  StringBuffer buffer;
  Writer<StringBuffer> writer(buffer);
  d.Accept(writer);

  // Output {"project":"rapidjson","stars":11}
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
