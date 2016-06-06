
//WIRELESS KILL SWITCH
//Receiver 7 to ch1
//Receiver 8 to ch3
//Receiver 6 to ch2
//

int pin_channel_wks1_ch1 = A0; //7 , 24V
int pin_channel_wks2_ch3 = A2; //8 ,12V
int pin_channel_pc_ch2 = A1; //6 ,miniPC


int pin_relay_wks1 = 1;
int pin_relay_wks2 = 0;
int pin_relay_pc = 9;


int var_store_wks1_ch1;
int var_store_wks2_ch3;
int var_store_pc_ch2;

int middle_ppm = 2000;

void setup(){

pinMode (pin_channel_wks1_ch1, INPUT);
pinMode (pin_channel_wks2_ch3, INPUT);
pinMode (pin_channel_pc_ch2, INPUT);

pinMode (pin_relay_wks1, OUTPUT);
pinMode (pin_relay_wks2, OUTPUT);
pinMode (pin_relay_pc, OUTPUT);

  Serial.begin(9600);

}

void loop()
{
  var_store_wks1_ch1 = pulseIn(pin_channel_wks1_ch1, HIGH);   //CH 7 RC

  if (var_store_wks1_ch1 > middle_ppm)
  {
    digitalWrite(pin_relay_wks1, HIGH);
  }
  else
  {
   digitalWrite(pin_relay_wks1, LOW);
  }


  var_store_wks2_ch3 = pulseIn(pin_channel_wks2_ch3, HIGH);  //CH 8 RC

  if (var_store_wks2_ch3 > middle_ppm ){
    digitalWrite(pin_relay_pc, HIGH);
  }
  else{
    digitalWrite(pin_relay_pc, LOW);
  }


  var_store_pc_ch2 = pulseIn(pin_channel_pc_ch2, HIGH);  //CH 6 RC

  if (var_store_pc_ch2 > middle_ppm){
    digitalWrite(pin_relay_wks2 , HIGH);
  }
  else{
    digitalWrite(pin_relay_wks2, LOW);
  }

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.print(var_store_wks1_ch1);
  Serial.print(" , ");
  Serial.print(var_store_wks2_ch3);
  Serial.print(" , ");
  Serial.println(var_store_pc_ch2);
}
