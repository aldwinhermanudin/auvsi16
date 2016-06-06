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

int sendTCP(string hostname, int port_number, string payload_data);

int main(int argc , char *argv[])
{
  string hostname = "127.0.0.1";
  int port_number = 80;
  string filename = "/data_receive.php";
  string payload = "port=1321";
  int payloadSize = payload.size();

  ostringstream oss;

  oss << "POST " << filename << " HTTP/1.1\n";
  oss << "Host: " << hostname << "\n";
  oss << "Content-length: " << payloadSize << "\n";
  oss << "Content-Type: application/x-www-form-urlencoded\n";
  oss << "Connection: close\n\n";
  oss << payload << "\n";
  cout << oss.str().c_str();

  sendTCP(hostname, port_number, oss.str());


  return 0;
}

int sendTCP(string hostname, int port_number, string payload){
  int socket_desc;
  struct sockaddr_in server;
  char server_reply[MAX_BUFFER];
  char message[MAX_BUFFER];

  //Create socket
  socket_desc = socket(AF_INET , SOCK_STREAM , 0);
  if (socket_desc == -1)
  {
    cout << "Could not create socket";
  }

  server.sin_addr.s_addr = inet_addr(hostname.c_str());
  server.sin_family = AF_INET;
  server.sin_port = htons( port_number );

  //Connect to remote server
  if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
  {
    cout << "connect error\n";
    return 1;
  }

  cout << "Connected\n";
  cout << "Message size " << payload.size() << endl;

  //Send some data
  if( send(socket_desc , payload.c_str() , payload.size(), 0) < 0)
  {
    cout << "Send failed";
    return 1;
  }
  cout << "Data Send\n";

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
  // At this point we have the available data (which may not be a complete
  // application level message).

  cout << "Reply received\n";

  std::stringstream ss;
  for(size_t i = 0; i < buffer.size(); ++i)
  {
    ss << buffer[i];
  }
  std::string s = ss.str();
  cout << s << endl;

  close(socket_desc);

  return 0;
}
