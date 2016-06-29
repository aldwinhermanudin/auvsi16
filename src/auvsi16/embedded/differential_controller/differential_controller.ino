// #### notes #### 
//
// left A4
// right A3
// override_apm A0 => 0 means led in PC817 on
// 
// run rosserial_python by using the command below
// rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
// rosrun rosserial_server serial_node _port:=/dev/ttyUSB0
//
// test in the cmd by using the command below
// rostopic pub -1 /auvsi16/differential_control auvsi16/differential_controller -- 2000 1100 false
// or
// rostopic pub -1 /auvsi16/differential_control auvsi16/differential_controller -- 2000 1100 0
//
// #### failure notes #### 
// rosserial_python or rosserial_server may fail connecting to arduino nano if datarate is to high.
// it seems that using rosserial_server with high baudrate causes error syncing

#include <Servo.h> 
#include <ros.h>
#include <auvsi16/differential_controller.h>

ros::NodeHandle  nh;
void motor_ctrl_cb( const auvsi16::differential_controller& recv_msg);
Servo left_motor,right_motor;
bool override_apm_flag;

// ### Pin Settings ###
int left_esc_pin = A4;
int right_esc_pin = A3;
int override_apm_pin = A0;
// ### Pin Settings ###

ros::Subscriber<auvsi16::differential_controller> mtr_ctrl_sub("auvsi16/differential_control", motor_ctrl_cb);
void setup(){
  pinMode(override_apm_pin, OUTPUT);
  
  // ### Initial condition : APM can control motor ###
  override_apm_flag = false;
  pinMode(left_esc_pin, INPUT);
  pinMode(right_esc_pin, INPUT);
  digitalWrite(override_apm_pin,LOW);
  // ### Initial condition : APM can control motor ###
  
  nh.getHardware()->setBaud(115200); // un-comment this to have a default baudrate (57600)
  nh.initNode();
  nh.subscribe(mtr_ctrl_sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}

void motor_ctrl_cb( const auvsi16::differential_controller& recv_msg){
   
  //Override APM Control
  if(recv_msg.override_apm){
    if(recv_msg.override_apm != override_apm_flag){
      left_motor.attach(left_esc_pin);
      right_motor.attach(right_esc_pin);
      digitalWrite(override_apm_pin,HIGH);
      override_apm_flag = recv_msg.override_apm;
    }
    left_motor.writeMicroseconds(recv_msg.left_pwm);
    right_motor.writeMicroseconds(recv_msg.right_pwm);
  }
  
  // APM Control
  else if(!recv_msg.override_apm && recv_msg.override_apm != override_apm_flag){
    left_motor.detach();
    right_motor.detach();
    pinMode(left_esc_pin, INPUT);
    pinMode(right_esc_pin, INPUT);
    digitalWrite(override_apm_pin,LOW);
    override_apm_flag = recv_msg.override_apm;
  }
}
