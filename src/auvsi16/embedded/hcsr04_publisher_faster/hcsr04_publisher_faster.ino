#include <NewPing.h>
#include <ros.h>
#include "auvsi16/sonarData.h"

#define SONAR_NUM     13 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(A1, A0, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A3, A2, MAX_DISTANCE), // L1
  NewPing(A5, A4, MAX_DISTANCE), // L2
  NewPing(A7, A6, MAX_DISTANCE), // L3
  NewPing(A11, A10, MAX_DISTANCE), // L4
  NewPing(A13, A12, MAX_DISTANCE), // L5
  NewPing(44, 46, MAX_DISTANCE), // L6
  
  NewPing(10, 11, MAX_DISTANCE), // R1
  NewPing(8, 9, MAX_DISTANCE), // R2
  NewPing(5, 6, MAX_DISTANCE), // R3
  NewPing(4, 3, MAX_DISTANCE), // R4
  NewPing(15, 14, MAX_DISTANCE), // R5
  NewPing(19, 18, MAX_DISTANCE)  // R6
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
    delay(2);    // this control the datarate of the hc-sr04
    snr_msg.data[i] = sonar[i].ping_cm();
    pub_sonar.publish( &snr_msg );
    nh.spinOnce();
  }
}
