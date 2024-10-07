#include <FlexCAN_T4.h>
#include <kinetis_flexcan.h>
#include <Wire.h>

/*Defining IDs*/
#define rpm_id 0x105
#define temp_id

/*Pin Mappings*/
const int LC_R = 17;
const int LC_L = 16;
const int Sus_R = 14;
const int Sus_L = 15;
const int LC_AR = 21;
const int LC_AL = 20;
const int temp_R;
const int temp_L;


/*Sensor Variable*/

uint16_t LC_R_in;
uint16_t LC_L_in;

uint16_t Sus_R_in;
uint16_t Sus_L_in;

uint16_t LC_AR_in;
uint16_t LC_AL_in;

uint16_t temp_R_in;
uint16_t temp_L_in;


/*Temp Variable*/
int calc_temp_R;
int calc_temp_L;
uint8_t temp[3];


/*CAN*/
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;
  
void setup() {
  pinMode(LC_R, INPUT);
  pinMode(LC_L, INPUT);
  pinMode(Sus_R, INPUT);
  pinMode(Sus_L, INPUT);
  pinMode(LC_AR, INPUT);
  pinMode(LC_AL, INPUT);
  /*Serial setup*/
  Serial.begin(9600);

  /*CAN setup*/
  can2.begin();
  can2.setBaudRate(1000000);
}


/*Functions*/ 
//Read sensor values
void read_data() {
  LC_R_in = analogRead(LC_R);
  LC_L_in = analogRead(LC_L);
  Sus_R_in = analogRead(Sus_R);
  Sus_L_in = analogRead(Sus_L);
  LC_AR_in = analogRead(LC_AR);
  LC_AL_in = analogRead(LC_AL);
  temp_R_in = analogRead(temp_R);
  temp_L_in = analogRead(temp_L);
}
//Temperature Calculation
void temp_calc(){
  temp[0] = calc_temp_R >> 2;
  temp[1] = ((calc_temp_R & 3) << 6) + (calc_temp_L >> 4);
  temp[2] = ((calc_temp_L & 15) >> 4);
}
//Storing data
void store_data(){
  msg.id = rpm_id;
  msg.buf[0] = LC_R_in & 255;
  msg.buf[1] = ((LC_L_in & 63) << 2) + (LC_R_in >> 8);
  msg.buf[2] = ((Sus_R_in & 15) << 4) + (LC_L_in >> 6);
  msg.buf[3] = ((Sus_L_in & 3) << 6) + (Sus_R_in >> 4);
  msg.buf[4] = Sus_L_in >> 2;
  msg.buf[5] = LC_AR & 255;
  msg.buf[6] = ((LC_AL & 63) << 2) + (LC_AR >> 8);
  msg.buf[7] = LC_AL >> 6;
}
//Writing Temperature data
void temp_request(){
  Wire.write(temp,3);
}
//Sending Temperature Data
void send_temp(){
  Wire.begin(temp_id);
  Wire.onRequest(temp_request);
}

void loop() {
  read_data();
  temp_calc();
  store_data();
  send_temp();
  can2.write(msg);
  Serial.println();
  Serial.println(Sus_R_in);
}
