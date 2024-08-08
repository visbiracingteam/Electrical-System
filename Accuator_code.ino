#include <FlexCAN_T4.h>

/*Defining IDs*/
#define apps_id 0x104

/*Pin Mappings*/
const int apps1 = 16;
const int apps2 = 14;
const int bps1 = 17;
const int bps2 = 15;
const int steer = 19;
const int sens6 = 18;


/*Sensor Variable*/
uint16_t apps1_in = 0;
uint16_t apps2_in = 0;
uint16_t bps1_in = 0;
uint16_t bps2_in = 0;
uint16_t steer_in = 0;
uint16_t sens6_in = 0;

/*Normalised variables*/
double apps1_normalised;
double apps2_normalised;
double apps_avg_frac;

/*Range of sensors*/
const uint16_t apps1_gmax = 1000;
const uint16_t apps1_gmin = 0;
const uint16_t apps2_gmax = 1000;
const uint16_t apps2_gmin = 0;

const uint16_t bps1_gmax = 1000;
const uint16_t bps1_gmin = 0;
const uint16_t bps2_gmax = 1000;
const uint16_t bps2_gmin = 0;

/*Working range of sensors (calibratiion)*/
const uint16_t apps1_max = 930;
const uint16_t apps1_min = 335;
const uint16_t apps2_max = 710;
const uint16_t apps2_min = 80;

const uint16_t apps1_range = apps1_max - apps1_min;
const uint16_t apps2_range = apps2_max - apps2_min;

const uint16_t bps1_max = 1000;
const uint16_t bps1_min = 0;
const uint16_t bps2_max = 1023;
const uint16_t bps2_min = 0;
const uint16_t bps1_range = bps1_max - bps1_min;
const uint16_t bps2_range = bps2_max - bps2_min;

/*CAN*/
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
//FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;


/*Functions*/
//Read sensor values
void read_data() {
  apps1_in = analogRead(apps1);
  apps2_in = analogRead(apps2);
  bps1_in = analogRead(bps1);
  bps2_in = analogRead(bps2);
  steer_in = analogRead(steer);
  sens6_in = analogRead(sens6);
}
//Storing data
void store_data(){
  msg.id = apps_id;
  msg.buf[0] = apps1_in & 255;
  msg.buf[1] = ((apps2_in & 63) << 2) + (apps1_in >> 8);
  msg.buf[2] = ((bps1_in & 15) << 4) + (apps2_in >> 6);
  msg.buf[3] = ((bps2_in & 3) << 6) + (bps1_in >> 4);
  msg.buf[4] = bps2_in >> 2;
  msg.buf[5] = steer_in & 255;
  msg.buf[6] = ((sens6_in & 63) << 2) + (steer_in >> 8);
  msg.buf[7] = sens6_in >> 6;
}

//Normalisation of values
void normalisation(){
  apps1_normalised = 1.0*(apps1_in - apps1_min)/apps1_range;
 // Serial.println(apps1_normalised);
  apps2_normalised = 1.0*(apps2_in - apps2_min)/apps2_range;
  //Serial.println(apps2_normalised);
  apps_avg_frac = (apps1_normalised + apps2_normalised)/2;
  //Serial.println("APPS_AVG_FRAC : ");
  //Serial.print(apps_avg_frac);
}

//Optimisation of values
void optimisation(){
  if(apps1_in > apps1_max)
  apps1_in = apps1_max;
  if(apps1_in < apps1_min)
  apps1_in = apps1_min;
  if(apps2_in > apps2_max)
  apps2_in = apps2_max;
  if(apps2_in < apps2_min)
  apps2_in = apps2_min;

  if(bps1_in < bps1_min)
  bps1_in = bps1_min;
  if(bps2_in < bps2_min)
  bps2_in = bps2_min;
//  if(bps1_in > bps1_max)
//  bps1_in = bps1_max;
//  if(bps2_in > bps2_max)
//  bps2_in = bps2_max;
}

void print_data(){
  Serial.print("APPS1_in ");
  Serial.println(apps1_in);
Serial.print("APPS2_in ");
   Serial.println(apps2_in);
  Serial.print(" BPS1_in ");
  Serial.println(bps1_in);
 Serial.print(" BPS2_in ");
 Serial.println(bps2_in);
 Serial.print(" Steer_in ");
 Serial.println(steer_in);
//  Serial.print(" Sens6_in ");
//  Serial.println(sens6_in);
//  Serial.print(" ID ");
//  Serial.println(msg.id);
  // Serial.print("APPS1 Pedal Position: ");
  // Serial.println(apps1_normalised);
  // Serial.print("APPS2 Pedal Position: ");
  // Serial.println(apps2_normalised);
  // Serial.print("Pedal Position Difference: ");
  // Serial.println(apps2_normalised - apps1_normalised);
}

void setup() {

  pinMode(apps1,INPUT);
  pinMode(apps2,INPUT);
  pinMode(bps1,INPUT);
  pinMode(bps2,INPUT);
  pinMode(steer,INPUT);
  pinMode(sens6,INPUT);
//  pinMode(19,INPUT);
  /*Serial setup*/
  Serial.begin(9600);

  /*CAN setup*/
  can2.begin();
  can2.setBaudRate(500000);
  
//  can1.begin();
//  can1.setBaudRate(500000);
}

void loop() {
  read_data();
  store_data();
  msg.id = apps_id;
  can2.write(msg);
//  optimisation();
 normalisation();
  print_data();
  delay(100);
}
