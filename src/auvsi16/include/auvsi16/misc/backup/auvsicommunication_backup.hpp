#include <vector>
#include <sstream>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>
#include <iostream>
#include <netinet/in.h>
#include <ctype.h>
#include <iomanip>

using namespace std;

enum communicationType {obstacle, docking, interoperability, pinger, heartbeat, start_run, end_run};

// Not yet implemented code for adding Interop and and pinger
// ########################################## start of auvsiCommunication class ########################################## //

class auvsiCommunication {

private:
  string payload;
  string response_message = "No data";
  string hostname;
  int port_number;
  int gps_precision = 9;
  string course_type;
  string team_code;
  communicationType comm_type;
  int receive_timeout = 10;

public:

  auvsiCommunication            (string hostname_input, int port_number_input, string course_type, string team_code, communicationType comm_type);
  int sendTCP                   ();
  string getPayload             ();
  string getRespone             ();
  bool setRemoteTarget          (string hostname_input, int port_number_input);
  bool setPayload               (string input_data);
  void setReceiveTimeout        (int receive_timeout);
  void setCommType              (communicationType comm_type);
  void setGPSPrecision          (int float_precision);
  void setCourseType            (string course_type);
  void setTeamCode              (string team_code);
  bool setPayloadCommunication  ();
  bool setPayloadCommunication  (string timestamp_mission, string challenge, float latitude, float longitude);
  int decodeResponeStatus       ();

};

auvsiCommunication::auvsiCommunication(string hostname, int port_number, string course_type, string team_code, communicationType comm_type){
  this->hostname = hostname;
  this->port_number = port_number;
  this->payload = "No data";
  this->course_type = course_type;
  this->team_code = team_code;
  this->comm_type = comm_type;
}

bool auvsiCommunication::setRemoteTarget(string hostname_input, int port_number_input){
  hostname = hostname_input;
  port_number = port_number_input;
}

bool auvsiCommunication::setPayload (string input_data){
  payload = input_data;
}

int auvsiCommunication::sendTCP(){

  // ################## create socket ################## //
  int socket_desc;
  struct sockaddr_in server;
  socket_desc = socket(AF_INET , SOCK_STREAM , 0);
  if (socket_desc == -1)
  {
    //cout << "Could not create socket";
    return -1;
  }
  server.sin_addr.s_addr = inet_addr(hostname.c_str());
  server.sin_family = AF_INET;
  server.sin_port = htons( port_number );
  // ################## create socket ################## //

  // ################## connect to remote server ################## //
  if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
  {
    //cout << "connect error\n";
    return -1;
  }

  //cout << "Connected\n";
  // ################## connect to remote server ################## //

  // ################## Send some data ################## //
  //cout << "Message size " << payload.size() << endl;
  if( send(socket_desc , payload.c_str() , payload.size(), 0) < 0)
  {
    //cout << "Send failed";
    return -1;
  }
  //cout << "Data Send\n";
  // ################## Send some data ################## //

  // ################## Receive some data ################## //
  // ################## set receive timout ################## //
  struct timeval tv;
	tv.tv_sec = receive_timeout;  /* 30 Secs Timeout */
	tv.tv_usec = 0;  // Not init'ing this can cause strange errors
  setsockopt(socket_desc, SOL_SOCKET, SO_RCVTIMEO, &tv,sizeof(tv));
  errno = 0;	// reset errno
  // ################## set receive timout ################## //

  // create the buffer with space for the data
  const unsigned int MAX_BUF_LENGTH = 4096;
  std::vector<char> buffer(MAX_BUF_LENGTH);
  std::string rcv;
  int bytesReceived = 0;
  do {
    bytesReceived = recv(socket_desc, buffer.data(), buffer.size(), 0);
    // append string from buffer.
    if ( bytesReceived == -1 ) {
      //cout << "recv failed";
      return -1;

    } else {
      rcv.append( buffer.cbegin(), buffer.cend() );
    }
  } while ( bytesReceived == MAX_BUF_LENGTH );

  if (errno == EAGAIN){
    return -1;
  }

  //cout << "Reply received\n";
  // ################## Receive some data ################## //

  // ################## Convert to string ################## //
  std::stringstream ss;
  for(size_t i = 0; i < buffer.size(); ++i){
    ss << buffer[i];
  }
  // ################## Convert to string ################## //

  close(socket_desc);

  response_message = ss.str();
  return 1;
}

string auvsiCommunication::getPayload (){

  return payload;
}

string auvsiCommunication::getRespone(){

  return response_message;
}

void auvsiCommunication::setReceiveTimeout(int receive_timeout){

  this->receive_timeout = receive_timeout;
}
void auvsiCommunication::setCommType(communicationType comm_type){

  this->comm_type = comm_type;
}

void auvsiCommunication::setTeamCode(string team_code){

  this->team_code = team_code;
}
void auvsiCommunication::setCourseType(string course_type){

  this->course_type = course_type;
}
void auvsiCommunication::setGPSPrecision(int gps_precision){

  this->gps_precision = gps_precision;
}

bool auvsiCommunication::setPayloadCommunication(){

  if (comm_type == docking || comm_type == obstacle ){
    ostringstream oss;
    string course_description;

    if (comm_type == obstacle){
      course_description = "obstacleAvoidance";
    } else if (comm_type == docking){
      course_description = "automatedDocking";
    }

    string filename;
    filename.append("/" + course_description + "/" + course_type + "/" + team_code);
    oss << "GET "<< filename <<" HTTP/1.1\r\n";
    oss << "Host: " << hostname << ":" << port_number<< "\r\n";
    oss << "Accept: */*\r\n\r\n";
    payload = oss.str();
    return true;
  }

  else if (comm_type == start_run || comm_type == end_run ){
    ostringstream oss;
    string course_description = "empty";

    if (comm_type == start_run){
      course_description = "start";
    } else if (comm_type == end_run){
      course_description = "end";
    }

    string filename;
    filename.append("/run/" + course_description + "/" + course_type + "/" + team_code);
    oss << "POST "<< filename <<" HTTP/1.1\r\n";
    oss << "Host: " << hostname << ":" << port_number<< "\r\n";
    oss << "Content-Length: 0\r\n";
    oss << "Content-Type: application/json\r\n\r\n";

    payload = oss.str();
    return true;
  }

  return false;
}

bool auvsiCommunication::setPayloadCommunication(string timestamp_mission, string challenge, float latitude, float longitude){
  if (comm_type == heartbeat){

    string filename = "/heartbeat/" + course_type + "/";
    filename.append(team_code);
    ostringstream payload_stream;
    payload_stream << "{\"timestamp\":\"" << timestamp_mission << "\",";
    payload_stream << "\"challenge\":\"" << challenge << "\",";
    payload_stream << "\"position\":{\"datum\":\"WGS84\",\"latitude\":"<< std::setprecision(gps_precision) << latitude << ",\"longitude\":" << std::setprecision(gps_precision) << longitude << "}}";
    int payload_size = payload_stream.str().size();

    ostringstream oss;
    oss << "POST "<< filename <<" HTTP/1.1\r\n";
    oss << "Host: " << hostname << ":" << port_number<< "\r\n";
    oss << "Content-Length: "<< payload_size << "\r\n";
    oss << "Content-Type: application/json\r\n\r\n";
    oss << payload_stream.str() << "\r\n";

    payload = oss.str();
    return true;
  }
  return false;
}

int auvsiCommunication::decodeResponeStatus(){
  std::string status_code;

  if(comm_type != interoperability)
    if (response_message.find("HTTP/1.1 200") != std::string::npos) {
      return 200;
    }
    else if (response_message.find("HTTP/1.1 400") != std::string::npos) {
      return 400;
    }
    else if (response_message.find("HTTP/1.1 404") != std::string::npos) {
      return 404;
    }
    else if (response_message.find("HTTP/1.1 500") != std::string::npos) {
      return 500;
    }
    else if (response_message.find("HTTP/1.1 503") != std::string::npos) {
      return 503;
    }
    else {
      return -1;
    }
  }



// ########################################## end of auvsiCommunication class ########################################## //

// ########################################## start of getTime class ########################################## //
class getTime {

  private:
    time_t t;
    struct tm * now;

  public:

    getTime ();
    void refreshTime();
    string getYear();
    string getMonth();
    string getDay();
    string getHour();
    string getMinute();
    string getSecond();
    string getYMDHS();
};

getTime::getTime(){
  t = time(0);   // get time now
  now = localtime( & t );
}
void getTime::refreshTime(){
  t = time(0);   // get time now
  now = localtime( & t );
}

string getTime::getYear(){
  this->refreshTime();
  int year = now->tm_year + 1900;
  return to_string(year);
}

string getTime::getMonth(){
  this->refreshTime();
  int month = now->tm_mon + 1;
  if(month < 10){
    return "0" + to_string(month);
  }
  else {
    return to_string(month);
  }
}

string getTime::getDay(){
  this->refreshTime();
  int day = now->tm_mday;
  if(day < 10){
    return "0" + to_string(day);
  }
  else {
    return to_string(day);
  }
}
string getTime::getHour(){
  this->refreshTime();
  int hour = now->tm_hour;
  if(hour < 10){
    return "0" + to_string(hour);
  }
  else {
    return to_string(hour);
  }
}
string getTime::getMinute(){
  this->refreshTime();
  int minute = now->tm_min;
  if(minute < 10){
    return "0" + to_string(minute);
  }
  else {
    return to_string(minute);
  }
}
string getTime::getSecond(){
  this->refreshTime();
  int second = now->tm_sec;
  if(second < 10){
    return "0" + to_string(second);
  }
  else {
    return to_string(second);
  }
}

string getTime::getYMDHS(){
  string complete_date = this->getYear() + this->getMonth() + this->getDay() + this->getHour() + this-> getMinute() + this->getSecond();
  return complete_date;
}
// ########################################## end of getTime class ########################################## //
