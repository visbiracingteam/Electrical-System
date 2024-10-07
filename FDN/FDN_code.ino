#include <FlexCAN_T4.h>
#include <kinetis_flexcan.h>
#include <Wire.h>

/*Defining IDs*/
#define rpm_id 0x302
#define temp_id

/*Pin Mappings*/
const int LC_R = 17;
const int LC_L = 16;
const int Sus_R = 14;
const int Sus_L = 15;
const int RPM_R = 21;
const int RPM_L = 20;
const int temp_R;
const int temp_L;


/*Sensor Variable*/

uint16_t LC_R_in = 0;
uint16_t LC_L_in = 0;

uint16_t Sus_R_in = 0;
uint16_t Sus_L_in = 0;

uint16_t RPM_R_in = 0;
uint16_t RPM_L_in = 0;

uint16_t temp_R_in = 0;
uint16_t temp_L_in = 0;


/*RPM Variables*/
int rpm_delay;
int teeth = 9;
uint16_t count_R = 0;
uint16_t count_L = 0;
uint16_t calc_RPM_R = 0;
uint16_t calc_RPM_L = 0;
unsigned long int start_time = 0;
unsigned long int curr_time = 0;
unsigned long int total_time = 0;


/*Temp Variable*/
int calc_temp_R = 0;
int calc_temp_L = 0;
uint8_t temp[3];


/*CAN*/
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;

void setup() {
  /*Serial setup*/
  Serial.begin(9600);

  /*RPM Interrupts*/
  attachInterrupt(digitalPinToInterrupt(RPM_R),counter_R,FALLING);
  attachInterrupt(digitalPinToInterrupt(RPM_L),counter_L,FALLING);

  /*CAN setup*/
  can2.begin();
  can2.setBaudRate(1000000);
}

void print_data() {
  Serial.print("Sus_R_in ");
  Serial.println(Sus_R_in);
  Serial.println();
  Serial.print("Sus_L_in ");
  Serial.print(Sus_L_in);
  Serial.println();
}

/*Functions*/
//Read sensor values
void read_data() {
  Serial.print("Curr_time2:");
  Serial.println(curr_time);
  Serial.print("start_time2:");
  Serial.println(start_time);
  LC_R_in = analogRead(LC_R);
  LC_L_in = analogRead(LC_L);
  Sus_R_in = analogRead(Sus_R);
  Sus_L_in = analogRead(Sus_L);
//  temp_R_in = analogRead(temp_R);
//  temp_L_in = analogRead(temp_L);
}
//Counting teeths
void counter_R(){
  count_R += 1;
  Serial.print("R:");
  Serial.println(count_R);
}
void counter_L(){
  Serial.print("hi");
  count_L += 1;
  Serial.print("L:");
  Serial.println(count_L);
}
void count_reset(){
  // Serial.print("Curr_time1:");
  // Serial.println(curr_time);
  // Serial.print("start_time1:");
  // Serial.println(start_time);
  count_L = 0;
  count_R = 0;
}
//RPM Calculation
void rpm_calc(){
  //delay(50);
  curr_time = millis();
  // Serial.print("Curr_time3:");
  // Serial.println(curr_time);
  // Serial.print("start_time3:");
  // Serial.println(start_time);
  calc_RPM_R = int((60000.0*count_R)/(teeth*(curr_time - start_time)));
  calc_RPM_L = int((60000.0*count_L)/(teeth*(curr_time - start_time)));
  start_time = millis();
}
//Temperature Calculation
void temp_calc(){
  temp[0] = calc_temp_R >> 2;
  temp[1] = ((calc_temp_R & 3) << 6) + (calc_temp_L >> 4);
  temp[2] = ((calc_temp_L & 15) >> 4);
}
//Storing data
void store_data(){
  // Serial.print("Curr_time4:");
  // Serial.println(curr_time);
  // Serial.print("start_time4:");
  // Serial.println(start_time);
  msg.id = rpm_id;
  msg.buf[0] = LC_R_in & 255;
  msg.buf[1] = ((LC_L_in & 63) << 2) + (LC_R_in >> 8);
  msg.buf[2] = ((Sus_R_in & 15) << 4) + (LC_L_in >> 6);
  msg.buf[3] = ((Sus_L_in & 3) << 6) + (Sus_R_in >> 4);
  msg.buf[4] = Sus_L_in >> 2;
  msg.buf[5] = calc_RPM_R & 255;
  msg.buf[6] = ((calc_RPM_L & 15) << 4) + (calc_RPM_R >> 8);
  msg.buf[7] = calc_RPM_L >> 4;
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
  count_reset();
  read_data();
  rpm_calc();
//  temp_calc();
  store_data();
//  send_temp();
Serial.print("calc_RPM_R:");
Serial.println(calc_RPM_R);
Serial.print("calc_RPM_L:");
Serial.println(calc_RPM_L);
Serial.print("L:");
Serial.println(count_L);
Serial.print("R:");
Serial.println(count_R);
  can2.write(msg);
  //print_data();
  //total_time = millis();
  // Serial.print("Curr_time:");
  // Serial.println(curr_time);
  // Serial.print("Start_time:");
  // Serial.println(start_time);
  // Serial.print("Total_time:");
  // Serial.println(total_time);
  // Serial.println(calc_RPM_R);
  // Serial.println(calc_RPM_L);
  delay(50);
}
