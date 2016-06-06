#include "include/auvsicommunication.hpp"

#define	MAX_BUFFER 3000
using namespace std;

int main(int argc , char *argv[])
{

  string hostname = "127.0.0.1";
  int port_number = 80;

  auvsiCommunication auvsi_protocol(hostname, port_number, "courseA", "rooster.php");
  auvsi_protocol.setPayloadCommunication(heartbeat, "20111995", "bomb", -123.231234,412.21234);
  cout << auvsi_protocol.getPayload();
  while (1)
  cout << auvsi_protocol.sendTCP();

  return 0;
}


  /*
  * PHP Test File - json_test.php

  <?php

  $request_body = file_get_contents('php://input');
  var_dump(json_decode($request_body, true));

  ?>

  */
