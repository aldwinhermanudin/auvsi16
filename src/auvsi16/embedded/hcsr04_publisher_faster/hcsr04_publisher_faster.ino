#include <NewPing.h>
#include <ros.h>
#include "auvsi16/sonarData.h"

#define SONAR_NUM     13 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(A1, A0, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A3, A2, MAX_DISTANCE),
  NewPing(A5, A4, MAX_DISTANCE)
};

int i;
ros::NodeHandle  nh;
auvsi16::sonarData snr_msg;
ros::Publisher pub_sonar("auvsi16/sonar_data", &snr_msg);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_sonar);
}

void loop() {
  for (i = 0; i < SONAR_NUM; i++){
    delay(5);    // this control the datarate of the hc-sr04
    snr_msg.data[i] = sonar[i].ping_cm();
    pub_sonar.publish( &snr_msg );
    nh.spinOnce();
  }
}
