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

// Not yet implemented code for adding Interop and

class auvsiCommunication {

private:
  string payload;
  string hostname;
  int port_number;
  int gps_precision = 9;
  string course_type;
  string team_code;

public:

  auvsiCommunication            (string hostname_input, int port_number_input, string course_type, string team_code);
  string sendTCP                ();
  string getPayload             ();
  bool setRemoteTarget          (string hostname_input, int port_number_input);
  bool setPayload               (string input_data);
  void setGPSPrecision          (int float_precision);
  void setCourseType            (string course_type);
  void setTeamCode              (string team_code);
  bool setPayloadCommunication  (communicationType comm_type);
  bool setPayloadCommunication  (communicationType comm_type, string timestamp_mission, string challenge, float latitude, float longitude);
};

  auvsiCommunication::auvsiCommunication(string hostname, int port_number, string course_type, string team_code){
    this->hostname = hostname;
    this->port_number = port_number;
    this->payload = "No data";
    this->course_type = course_type;
    this->team_code = team_code;
  }

  bool auvsiCommunication::setRemoteTarget(string hostname_input, int port_number_input){
    hostname = hostname_input;
    port_number = port_number_input;
  }

  bool auvsiCommunication::setPayload (string input_data){
    payload = input_data;
  }

  string auvsiCommunication::sendTCP(){

    // ################## create socket ################## //
    int socket_desc;
    struct sockaddr_in server;
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
      cout << "Could not create socket";
      return "-1";
    }
    server.sin_addr.s_addr = inet_addr(hostname.c_str());
    server.sin_family = AF_INET;
    server.sin_port = htons( port_number );
    // ################## create socket ################## //

    // ################## connect to remote server ################## //
    if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
      cout << "connect error\n";
      return "-1";
    }

    cout << "Connected\n";
    // ################## connect to remote server ################## //

    // ################## Send some data ################## //
    cout << "Message size " << payload.size() << endl;
    if( send(socket_desc , payload.c_str() , payload.size(), 0) < 0)
    {
      cout << "Send failed";
      return "-1";
    }
    cout << "Data Send\n";
    // ################## Send some data ################## //

    // ################## Receive some data ################## //
    // create the buffer with space for the data
    const unsigned int MAX_BUF_LENGTH = 4096;
    std::vector<char> buffer(MAX_BUF_LENGTH);
    std::string rcv;
    int bytesReceived = 0;
    do {
      bytesReceived = recv(socket_desc, buffer.data(), buffer.size(), 0);
      // append string from buffer.
      if ( bytesReceived == -1 ) {
        cout << "recv failed";
        // error
      } else {
        rcv.append( buffer.cbegin(), buffer.cend() );
      }
    } while ( bytesReceived == MAX_BUF_LENGTH );

    cout << "Reply received\n";
    // ################## Receive some data ################## //

    // ################## Convert to string ################## //
    std::stringstream ss;
    for(size_t i = 0; i < buffer.size(); ++i)
    {
      ss << buffer[i];
    }
    // ################## Convert to string ################## //

    close(socket_desc);

    return ss.str();
  }

  string auvsiCommunication::getPayload (){

    return payload;
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

  bool auvsiCommunication::setPayloadCommunication(communicationType comm_type){

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

  bool auvsiCommunication::setPayloadCommunication(communicationType comm_type, string timestamp_mission, string challenge, float latitude, float longitude){
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
