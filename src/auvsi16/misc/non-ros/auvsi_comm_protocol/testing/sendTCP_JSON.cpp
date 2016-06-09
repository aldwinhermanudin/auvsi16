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

#define	MAX_BUFFER 3000
using namespace std;

string sendTCP(string hostname, int port_number, string payload_data);

int main(int argc , char *argv[])
{

  string hostname = "127.0.0.1";
  int port_number = 80;
  string filename = "/json_test.php";
  string payload = "{\"course\":\"courseA\",\"team\":\"AUVSI\",\"buoyColor\":\"black\"}";
  int payloadSize = payload.size();

  ostringstream oss;

  oss << "POST "<< filename <<" HTTP/1.1\r\n";
  oss << "Host: " << hostname << "\r\n";
  oss << "Content-Length: "<< payloadSize << "\r\n";
  oss << "Content-Type: application/json\r\n\r\n";
  oss << payload << "\r\n";
  cout << oss.str().c_str();

  /*
  * PHP Test File - json_test.php

  <?php

  $request_body = file_get_contents('php://input');
  var_dump(json_decode($request_body, true));

  ?>

  */

  cout << sendTCP(hostname, port_number, oss.str());

  return 0;
}

string sendTCP(string hostname, int port_number, string payload){

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
