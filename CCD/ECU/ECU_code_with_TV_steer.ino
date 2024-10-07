#include <FlexCAN_T4.h>

/*Defining IDs*/
#define apps_id 0x104 // receive from apps at this id
#define ams_id 1314 // receive from ams at this id
#define rpm_id 120 // receive from rpm sensor at this id
#define yaw_id 112 // receive from yaw at this id
#define dash_sent_id 169 // receive from dash at this id
#define mc_left_recieve 0x202 // Left mc recieves data on this
#define mc_right_recieve 0x201 // Right mc recieves data on this
#define mc_left_transmit 0x182 // Left mc transmits message through this 
#define mc_right_transmit 0x181 // Right mc transmits mesaage through this

//Pin mappinigs
const int rtds_sig = 4;  //triggers RTDS
const int br_light = 2;  // triggers brake light
const int ignition_sig = 20;  // receives signal from ignition switch
const int dr_select = 21; // driver mode select
const int ECU_error = 3;  // triggers ECU error (shutdown circuit)
const int AB = 12; // unsued
const int AD = 5;  // unused
const int AE = 9;  // unused
const int c_plus = 18;  // receives C plus signal from FL
const int frg = 6; // Enable signal to be sent to MCs after completion of ignitiion sequence (frg first) hold high until all good
const int rfe = 7;   // Enable signal to be sent to MCs after completion of ignitiion sequence (after frg) hold high until all good

/*Start-up variables*/
//uint16_t ignition;
bool rtds = false; //rtds status flag

/*Sensor varialble*/
uint16_t apps1_in; // stores received apps1 value
uint16_t apps2_in; // stores received apps2 value

uint16_t steer_in;

uint16_t bps1_in; // stores received bps1 value
uint16_t bps2_in; // stores received bps1 value
const uint16_t bps_thr = 15; // Enter threshold for tiggering brake light in bar(*calibrate here)
uint16_t bps_threshold = 150; //Only defining variable here, will be updated using bps_thr abd pressure_mapper function
float pressure_mapper(int pressure){ //takes input in form of bar and returns value in between 0 to 1024
  return ((((pressure*4)/138)+0.5)*(1024/5));
}

/*Normalised variables*/
double apps1_normalised; //mapped value of apps1 between 0 to 1 according to calibrated value
double apps2_normalised; //mapped value of apps2 between 0 to 1 according to calibrated value
double apps_avg_frac; // average value of apps1_normalized and apps2_normalized

double steer_normalised;

/*Plausibility variables*/
//BPS
const uint16_t bps_plaus = 80; // unused
uint32_t timer_hb = 0; // unused

//APPS
uint32_t timer_10percent = 0; // for how much time both apps values are out of the 10 percent limit
uint32_t timer_apps_equal = 0; // for how much time both apps values are equal limit

/*Flags Initialisation*/
//BPS
bool hb_flag = false; // unused

//APPS
bool apps_10percent_flag = false;
bool apps_equal_flag = false;

/*Range of sensors*/
const uint16_t apps1_gmax = 1000; //Value of apps1 greater than gmax then apps_range_error will trigger, this means the value of apps is pulled up
const uint16_t apps1_gmin = 20; //Value of apps1 less than gmin then apps_range_error will trigger, this means the value of apps is pulled down or shorted to ground
const uint16_t apps2_gmax = 1000; //Value of apps2 greater than gmax then apps_range_error will trigger, this means the value of apps is pulled up
const uint16_t apps2_gmin = 20; //Value of apps2 less than gmin then apps_range_error will trigger, this means the value of apps is pulled down or shorted to ground

const uint16_t bps1_gmax = 925; //Value of bps1 greater than gmax then bps_range_error will trigger, this means the value of bps is pulled up
const uint16_t bps1_gmin = 80; //Value of bps1 less than gmin then bps_range_error will trigger, this means the value of bps is pulled down or shorted to ground
const uint16_t bps2_gmax = 925; //Value of bps2 greater than gmax then bps_range_error will trigger, this means the value of bps is pulled up
const uint16_t bps2_gmin = 80; //Value of bps2 less than gmin then bps_range_error will trigger, this means the value of bps is pulled down or shorted to ground

const uint16_t steer_gmax = 1000;
const uint16_t steer_gmin = 0;

/*Working range of sensors (calibratiion)*/
const uint16_t apps1_max = 680; //max value used for calibration
const uint16_t apps1_min = 55; //min value used for calibration
const uint16_t apps2_max = 910; //max value used for calibration
const uint16_t apps2_min = 300; //min value used for calibration
const uint16_t apps1_range = apps1_max - apps1_min; //range defined, used to find the normalized value
const uint16_t apps2_range = apps2_max - apps2_min; //range defined, used to find the normalized value
 
const uint16_t bps1_max = 1000; //max value used for calibration
const uint16_t bps1_min = 0; //max value used for calibration
const uint16_t bps2_max = 1023; //max value used for calibration
const uint16_t bps2_min = 0; //max value used for calibration
const uint16_t bps1_range = bps1_max - bps1_min; //range defined, used to find the normalized value
const uint16_t bps2_range = bps2_max - bps2_min; //range defined, used to find the normalized value

const uint16_t neutral_steer = 1000;
const uint16_t max_steer_right = 0;
const uint16_t max_steer_left = 0;

//AMS Data
double deration_factor = 0.75;

/*CAN buses initialisation*/
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> c_can;  // Control CAN bus
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> d_can;  // Data CAN bus
CAN_message_t msg;   //CAN message structure
//Reading sensor values
void sensor_read(const CAN_message_t &msg){
    if(msg.id == apps_id){
    
    apps1_in = ((msg.buf[1] & 3) << 8) + msg.buf[0];
    apps2_in = ((msg.buf[2] & 15) << 6) + (msg.buf[1] >> 2);
    bps1_in = ((msg.buf[3] & 63) << 4) + (msg.buf[2] >> 4);
    bps2_in = (msg.buf[4] << 2) + (msg.buf[3] >> 6);
    steer_in = ((msg.buf[6] & 3) << 8) + msg.buf[5];
    }
}

//Checking global range
bool apps_range_error(){
  if((apps1_in < apps1_gmin) || (apps1_in > apps1_gmax) || (apps2_in < apps2_gmin) || (apps2_in > apps2_gmax) )
    return true;
  else
    return false;
}

bool bps_range_error(){
  if((bps1_in < bps1_gmin) || (bps1_in > bps1_gmax) || (bps2_in > bps2_gmax) || (bps2_in < bps2_gmin))
    return true;
  else
    return false;
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
//steer_in also needed
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

  steer_normalised = (steer_in - neutral_steer)/(max_steer_right - neutral_steer);
}


/* Torque calculation variables*/
//bool send_torque_cmd = false;
double torque_avg = 0;
double torque_right = 0;
double torque_left = 0;
double delta_torque = 0;
int torque_left_number=0;
int torque_right_number=0;

/* MC Receive Variables*/
int enable_receive_left;
int enable_receive_right;

double rpm_right_receive;
double rpm_left_receive;

double torque_right_receive;
double torque_left_receive;

double voltage_right_receive;
double voltage_left_receive;

double power_right_receive;
double power_left_receive;

double current_right_receive;
double current_left_receive;

int motor_temp_right_receive;
int motor_temp_left_receive;

int mc_temp_right_receive;
int mc_temp_left_receive;


/* MC enable Flags */
int disable_send_flag_1=0;
int disable_send_flag_2=0;

int enable_request_flag_1 =0;
int enable_request_flag_2 =0;

int enable_sent_left = 0;
int enable_sent_right= 0;

/*Motor initial setup variables*/
//const char BTB_request[8]= {0x3D,0xE2,0,0,0,0,0,0};
const char disable_send[8]= {0x51,0x04,0,0,0,0,0,0};
const char enable_send[8]= {0x51,0,0,0,0,0,0,0};

const char enable_request[8]= {0x3D,0xE8,0x64,0,0,0,0,0}; // 1
const char speed_request[8]= {0x3D,0x30,0xC8,0,0,0,0,0}; //30 ms cyclic // 2
const char torque_request[8]= {0x3D,0xA0,0xC8,0,0,0,0,0}; //30 ms cyclic // 3
const char voltage_request[8]= {0x3D,0xEB,0xC8,0,0,0,0,0}; //30 ms cyclic // 4
const char power_request[8]= {0x3D,0xF6,0xC8,0,0,0,0,0}; //30 ms cyclic // 5
const char current_request[8]= {0x3D,0x20,0xC8,0,0,0,0,0}; //30 ms cyclic // 6
const char motor_temp_request[8]= {0x3D,0x49,0xC8,0,0,0,0,0}; //30 ms cyclic // 7
const char mc_temp_request[8]= {0x3D,0x4A,0xC8,0,0,0,0,0}; //30 ms cyclic // 8
//const char acc_ramp_send[8]= {0x35,0,0,0,0,0,0,0};
//const char dec_ramp_send[8]= {0xED,0,0,0,0,0,0,0};

// MC CAN receive, RPM = 0x31, torque = 0x90

char torque_send_left[8]= {0x90,0,0,0,0,0,0,0};
char torque_send_right[8]= {0x90,0,0,0,0,0,0,0};

/* Variables for calculating torque number (Method 1)*/
const double I_max_pk = 169.8; // 100% in NDrive; must be same for both MCs
const double torque_per_current = 0.5;
double I_des_Arms_left=0;
double I_des_Amps_left=0;
double I_des_Arms_right=0;
double I_des_Amps_right=0;

/*Variables for calculating torque number (Method 2)*/
const double ecu_max_torque = 40;  // Nm
const double motor_max_torque = 90;  // Nm
const double tv_max_torque = 4;


/* NDrive Parameters*/
//int torque_max_number = (32767*ecu_max_torque)/ndrive_max_torque;
//bool change_torque_status = true;
//int change_torque_timer = 0;

void initial_setup();

// Torque Command Functions
void send_left(){ //Actual right
   Serial.print("torque_left_number "); Serial.println(torque_left_number);
    torque_left_number = - torque_left_number;
    torque_send_left[2]=torque_left_number >> 8;
    torque_send_left[1]=torque_left_number & 255;
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = torque_send_left[i];
    msg.id = mc_left_recieve ;
    c_can.write(msg);
}

void send_right(){

    torque_right_number = torque_right_number;
    Serial.print("torque_right_number "); Serial.println(torque_right_number);
    torque_send_right[2]=torque_right_number >> 8;
    torque_send_right[1]=torque_right_number & 255;
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = torque_send_right[i];
    msg.id = mc_right_recieve;
    c_can.write(msg);
}

void mc_read(const CAN_message_t &msg){   
    if(msg.id == apps_id){
//      Serial.println("Sensor");
      // Serial.println("Haahahah");
      apps1_in = ((msg.buf[1] & 3) << 8) + msg.buf[0];
//      Serial.print(" APPS1_in ");Serial.println(apps1_in);
      apps2_in = ((msg.buf[2] & 15) << 6) + (msg.buf[1] >> 2);
//      Serial.print(" APPS2_in ");Serial.println(apps2_in);
      bps1_in = ((msg.buf[3] & 63) << 4) + (msg.buf[2] >> 4);
//      Serial.print(" APPS2_in ");Serial.println(bps1_in);
      bps2_in = (msg.buf[4] << 2) + (msg.buf[3] >> 6);
//    steer_in = ((msg.buf[6] & 3) << 8) + msg.buf[5];
    }
    else if(msg.id== mc_left_transmit){
//      Serial.println("Yes2");
      if(msg.buf[0] == 0x30){
        //reading speed (Speed actual value)
        rpm_left_receive = -(((msg.buf[2]<<8) | msg.buf[1] ));
      }
      else if(msg.buf[0] == 0xE8){ 
        //reading enable (Digital input RUN)
        enable_receive_left = msg.buf[1];
      }
      else if(msg.buf[0] == 0xA0){ 
        //reading torque (Digital torque inter)
        torque_left_receive = -(((msg.buf[2]<<8) | msg.buf[1] ));
      }
      else if(msg.buf[0] == 0xEB){
        //reading voltage (DC Bus Voltage)
        voltage_left_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
      else if(msg.buf[0] == 0xF6){ 
        //reading power
        power_left_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
      else if(msg.buf[0] == 0x20){ 
        //reading current (Current actual value)
        current_left_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
      else if(msg.buf[0] == 0x49){ 
        //reading motor temperature number
        motor_temp_left_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
      else if(msg.buf[0] == 0x4a){ 
        //reading mc temperature number
        mc_temp_left_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
    }
    else if(msg.id== mc_right_transmit){
//      Serial.println("Yes1");
      if(msg.buf[0] == 0x30){
        //reading speed
        rpm_right_receive = ( ( (msg.buf[2]<<8) | msg.buf[1] ) );
      }
      else if(msg.buf[0] == 0xE8){ 
        //reading enable
        enable_receive_right = msg.buf[1];
      }
      else if(msg.buf[0] == 0xA0){ 
        //reading torque
        torque_right_receive = (((msg.buf[2]<<8) | msg.buf[1] ));
      }
      else if(msg.buf[0] == 0xEB){ 
        //reading voltage
        voltage_right_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
      else if(msg.buf[0] == 0xF6){ 
        //reading power
        power_right_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
      else if(msg.buf[0] == 0x20){ 
        //reading current (Current actual value)
        current_right_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
      else if(msg.buf[0] == 0x49){ 
        //reading motor temperature number
        motor_temp_right_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
      else if(msg.buf[0] == 0x4a){ 
        //reading mc temperature number
        mc_temp_right_receive = ( ( msg.buf[2]<<8) | msg.buf[1] );
      }
    }
    // else if(msg.id == 0x400){
    //   Serial.println(msg.id);
    //   Serial.println(msg.buf[2]);

    // }
    else {
//      Serial.print("MC can not receive");
    }
}

void initial_setup(){
//  Serial.print("============= initial setup =============");
//   for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = disable_send[i];
//   msg.id = mc_left_recieve;
// //    if(SN_can.write(msg))
// //    {
// //        Serial.print("disable_left \n");
// //    }
//   for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = disable_send[i];
//   msg.id = mc_right_recieve;
//    if(SN_can.write(msg))
//    {
//        Serial.print("disable_right \n");
//    }

    // We can receive 8 variables value from each MC cyclically

    // 1. MC Enable Status Request (1 when HV reset is pressed)
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = enable_request[i];
    msg.id = mc_left_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("requested_left \n");
    }
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = enable_request[i];
    msg.id = mc_right_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("requested_right \n");
    }
    
    // 2. Speed (RPM) Request
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = speed_request[i];
    msg.id = mc_left_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("speed_left \n");
    }
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = speed_request[i];
    msg.id = mc_right_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("speed_right \n");
    }
    
    // 3. Torque Request
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = torque_request[i];
    msg.id = mc_left_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("torque_left \n");
    }
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = torque_request[i];
    msg.id = mc_right_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("torque_right \n");
    }
    
    // 4. Voltage Request
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = voltage_request[i];
    msg.id = mc_left_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("volt_left \n");
    }
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = voltage_request[i];
    msg.id = mc_right_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("volt_right \n");
    }
    
    // 5. Power Request
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = power_request[i];
    msg.id = mc_left_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("power_left \n");
    }
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = power_request[i];
    msg.id = mc_right_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("power_right \n");
    }
    
    // 6. Current Request
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = current_request[i];
    msg.id = mc_left_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("current_left \n");
    }
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = current_request[i];
    msg.id = mc_right_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("current_right \n");
    }
    
    // 7. Motor temperature request Request
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = motor_temp_request[i];
    msg.id = mc_left_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("motor temp request left \n");
    }
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = motor_temp_request[i];
    msg.id = mc_right_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("MC temp request right \n");
    }
   
    // 8. MC temperature Request
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = mc_temp_request[i];
    msg.id = mc_left_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("MC temp request left \n");
    }
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = mc_temp_request[i];
    msg.id = mc_right_recieve;
    if(c_can.write(msg))
    {
//        Serial.print("MC temp request right \n");
    }
//    Serial.print("exiting initial setup \n");
}
void initial_setup_new()
{
//    Serial.print("============= initial setup =============");
    // Replaced all if statements in initial_setup st: if(condition){print done} => while(!condition){print not done}; print done;
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = disable_send[i];
    msg.id = 0x201;
//    while(!SN_can.write(msg))
//    {
//        Serial.print("trying... ");
//    }
//    Serial.print("disable_left \n");
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = disable_send[i];
    msg.id = 0x202;
//    while(!SN_can.write(msg))
//    {
//        Serial.print("trying... ");
//    }
//    Serial.print("disable_right \n");

    // We can receive 8 variables value from each MC cyclically

    // 1. MC Enable Status Request (1 when HV reset is pressed)
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = enable_request[i];
    msg.id = 0x201;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("requested_left \n");
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = enable_request[i];
    msg.id = 0x202;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("requested_right \n");

    // 2. Speed (RPM) Request
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = speed_request[i];
    msg.id = 0x201;
    while(!c_can.write(msg))
    {
//       Serial.print("trying... ");
    }
//    Serial.print("speed_left \n");
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = speed_request[i];
    msg.id = 0x202;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("speed_right \n");

    // 3. Torque Request
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = torque_request[i];
    msg.id = 0x201;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("torque_left \n");
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = torque_request[i];
    msg.id = 0x202;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("torque_right \n");

    // 4. Voltage Request
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = voltage_request[i];
    msg.id = 0x201;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("volt_left \n");
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = voltage_request[i];
    msg.id = 0x202;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("volt_right \n");

    // 5. Power Request
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = power_request[i];
    msg.id = 0x201;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("power_left \n");
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = power_request[i];
    msg.id = 0x202;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("power_right \n");

    // 6. Current Request
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = current_request[i];
    msg.id = 0x201;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
    Serial.print("current_left \n");
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = current_request[i];
    msg.id = 0x202;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("current_right \n");

    // 7. Motor temperature request Request
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = motor_temp_request[i];
    msg.id = 0x201;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("motor temp request left \n");
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = motor_temp_request[i];
    msg.id = 0x202;
    while(!c_can.write(msg))
    {
//       Serial.print("trying... ");
    }
//    Serial.print("Motor temp request right \n");

    // 8. MC temperature Request
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = mc_temp_request[i];
    msg.id = 0x201;
    while(!c_can.write(msg))
    {
//        Serial.print("trying... ");
    }
//    Serial.print("MC temp request left \n");
    for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = mc_temp_request[i];
    msg.id = 0x202;
    while(!c_can.write(msg))
    {
        Serial.print("trying... ");
    }
//    Serial.print("MC temp request right \n");
//    Serial.print("exiting initial setup \n");
}

// Print MC data
void print_mc_data(){
    // // Print values from MC
    // // Enable
//    Serial.print("enable_receive_left "); Serial.println(enable_receive_left);
//    Serial.print("enable_receive_right "); Serial.println(enable_receive_right);
    // // Power
    // Serial.print("power_left_receive %f \n", power_left_receive);
    // Serial.print("power_right_receive %f \n", power_right_receive);
    // // Voltage
    // Serial.print("voltage_left_receive %f \n", voltage_left_receive);
    // Serial.print("voltage_right_receive %f \n", voltage_right_receive);
    // // Current
    // Serial.print("current_left_receive %f \n", current_left_receive);
    // Serial.print("current_right_receive %f \n", current_right_receive);
    // // Torque
    // Serial.print("torque_left_receive %f \n", torque_left_receive);
    // Serial.print("torque_right_receive %f \n", torque_right_receive);
    // // RPM
    // Serial.print("rpm_left_receive %f \n", rpm_left_receive);
    // Serial.print("rpm_right_receive %f \n", rpm_right_receive);
    // // Motor temperture
    // Serial.print("motor_temp_left_receive %d \n", motor_temp_left_receive);
    // Serial.print("motor_temp_right_receive %d \n", motor_temp_right_receive);
    // // MC temperature
    // Serial.print("mc_temp_left_receive %d \n", mc_temp_left_receive);
    // Serial.print("mc_temp_right_receive %d \n", mc_temp_right_receive);
}

/*Torque*/ 
//Torque Cap
void torque_cap(){
  torque_right = min(ecu_max_torque, max(torque_left, 0));
  torque_left = min(ecu_max_torque, max(torque_left, 0));
}
//Torque adujustment
void torque_adjustment(){
  //Method 1
  delta_torque = tv_max_torque*apps_avg_frac*steer_normalised;
  torque_left = torque_left + delta_torque;
  torque_right = torque_right - delta_torque;
  //Method 2
//  car_velocity = tire_radius * rpm_avg; //average
//  curr_slip_angle = (car_length * yaw_rate)/car_velocity - steer_angle;
//  adjusted_torque = kp * curr_slip_angle + kd * (curr_slip_angle - prev_slip_angle)/millis();//
//  // +ve steer is right turn and vice-versa
//  if(steer_angle > 0){
//    torque_R = calc_torque_R - adjusted_torque;
//    torque_L = calc_torque_L + adjusted_torque;
//  }
//  if(steer_angle < 0){
//    torque_R = calc_torque_R + adjusted_torque;
//    torque_L = calc_torque_L - adjusted_torque;
//  }
//  prev_slip_angle = curr_slip_angle;
}
void torque_deration(){
  torque_left = deration_factor*torque_left;
  torque_right = deration_factor*torque_right;;
}
//Torque calculation
void set_torque(){
    // Map 1
    torque_avg = ecu_max_torque*apps_avg_frac*1.0;
    torque_left = torque_avg;
    torque_right = torque_avg;
    //torque_adjustment();
    //torque_deration();
    torque_cap();
    //torque_left_number = 100;
    torque_left_number = 32767*(torque_left/motor_max_torque);
    //torque_right_number = 100;
    torque_right_number = 32767*(torque_right/motor_max_torque);
}

/*Brake*/
bool brake_pressed(){
  if((bps1_in > bps_threshold) || (bps_range_error()))
  {Serial.println(bps1_in);
  // delay(100);
  return true;
  Serial.println("Bps1 pressed");
  
  }
  else
  { 
  Serial.println("brakes not pressed");
  // delay(100);
  return false;
  
  }
}

/*Plausibility checks*/
//Hard Braking 
bool hard_braking(){
  if(bps1_in > bps_plaus && (apps_avg_frac > 0.25)){
    if(timer_hb == 0){
      timer_hb = millis();
      return false;
    }
    else if((millis()-timer_hb) > 500){
      return true;
    } 
  }
  else if(hb_flag == true && apps_avg_frac > 0.05){
    return true;
  }
  else{
    timer_hb = 0;
    return false;
  }
}
//APPS 10% Difference
bool apps_10percent(){
  if((apps1_normalised - apps2_normalised > 0.16) || ((apps1_normalised - apps2_normalised < -0.16))){
    if(timer_10percent == 0){
      timer_10percent = millis();
        return false;
    }
    else if((millis()-timer_10percent) > 500){
      return true;
    }
  }
  else{
    timer_10percent = 0;
    return false;
  }
}
//APPS Equalilty
bool apps_equal(){
  if(apps1_in == apps2_in){
    if(timer_apps_equal == 0){
      timer_apps_equal = millis();
      return false;
    }
    else if((millis()-timer_apps_equal) > 500){
      return true;
    }
  }
  else{
    timer_apps_equal = 0;
    return false;
  }
}

//Start-up
void start_up(){
//  ignition = digitalRead(ignition_sig);
  if(!rtds && (digitalRead(ignition_sig) == LOW) && brake_pressed()){ //Logic Reversed
//    Serial.println("Ignition done");
    digitalWrite(rtds_sig,HIGH);
    delay(2000);
    digitalWrite(rtds_sig, LOW);
    rtds = true;
  }
  else{
    digitalWrite(rtds_sig, LOW);
    rtds = false;
  }
}

/*rfe and frg on-off function*/
void on(){
  digitalWrite(rfe, HIGH);
  delay(0.5);
  digitalWrite(frg, HIGH);
}

void off(){
  digitalWrite(rfe, LOW);
  delay(1);
  digitalWrite(frg, LOW);
}

bool precharge(){
  if (digitalRead(c_plus) == LOW){//logic reversed
//    Serial.println("C_plus");
    on();
    return true;
  }
  else{
    off();
    rtds = false;
    return false;
  }
}

//void change_max_torque(){
//  if (digitalRead(dr_select) == LOW){
//    change_torque_timer = millis();
//    if(change_torque_status == true){
//      fixed_rpm_max += 500;
//    }
//    else{
//      fixed_rpm_max -= 500;
//    }
//  }
//  else if((millis() - change_torque_timer) > 4000){
//    change_torque_status = !(change_torque_status);
//    change_torque_timer = 0;
//  }
//  else{
//    change_torque_timer = 0;
//  }
//  if(change_torque_timer
//}

//bool shutdown(){
//  if(digitalRead(c_plus) == HIGH){
//    return true;
//  }
//  else{
//    return false;
//  }
//}


void setup() {
  /*Pins setup*/
  pinMode(ignition_sig, INPUT_PULLDOWN);
  pinMode(rtds_sig, OUTPUT);
  pinMode(br_light, OUTPUT);
  pinMode(frg,OUTPUT);
  pinMode(rfe,OUTPUT);
  pinMode(c_plus, INPUT_PULLDOWN);
//  pinMode(AD, OUTPUT);
//  pinMode(AE, OUTPUT);
//  pinMode(AB, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(c_plus), precharge, CHANGE);
//  pinMode(dr_select, INPUT_PULLDOWN);
//  attachInterrupt(digitalPinToInterrupt(dr_select), change_max_torque, CHANGE);
  
  /*Serial setup*/
  Serial.begin(11520);

  /*Can setup*/
  //Control CAN
  c_can.begin();
  c_can.setBaudRate(500000);
  c_can.enableFIFO();
  c_can.enableFIFOInterrupt();
  c_can.onReceive(mc_read);
  Serial.println("Hello");

  delay(3000);
  initial_setup();
}

void loop(){
  // apps_10percent_flag = apps_10percent();
  // if(apps_10percent_flag){
  //         Serial.println("APPS difference more than 10%");
  // }
  bps_threshold = pressure_mapper(bps_thr);
  bool dummy = brake_pressed();
  if(dummy){
    
    digitalWrite(br_light, HIGH);
    // Serial.println("brakes light on Commanded");
    // delay(3000);
  }
  else{
    digitalWrite(br_light, LOW);
    // Serial.println("brakes light off Commanded");
    // delay(3000);
  }
  
  delay(10);

  if(!(apps_range_error()||bps_range_error())){
    optimisation();
    normalisation();
    if(precharge()){
      start_up();
      if(rtds){
        hb_flag = hard_braking();
        apps_10percent_flag = apps_10percent();
        apps_equal_flag = apps_equal();
//         if(hb_flag){
//           Serial.println("Release the brakes and accelerator");
//           torque_left_number = 0;
//           torque_right_number = 0;
// //          digitalWrite(AD,LOW);
// //          digitalWrite(AE,LOW);
// //          digitalWrite(AB,HIGH);
//         }
         if(apps_10percent_flag){
          Serial.println("APPS difference more than 10%");
          torque_left = 0;
          torque_right = 0;
//          digitalWrite(AD,LOW);
//          digitalWrite(AE,HIGH);
//          digitalWrite(AB,HIGH);
        }
        else if(apps_equal_flag){
          Serial.println("Put some difference in APPS");
          torque_left = 0;
          torque_right = 0;
//          digitalWrite(AD,LOW);
//          digitalWrite(AE,HIGH);
//          digitalWrite(AB,LOW);
        }
        else{
          print_mc_data();
          for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = enable_send[i];
          msg.id = 0x201;
          c_can.write(msg);
          for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = enable_send[i];
          msg.id = 0x202; 
          c_can.write(msg);
          enable_sent_right = 1;
          enable_sent_left = 1;
          set_torque();
//          set_torque();
          //Serial.println("Be Happy Arush");
//          digitalWrite(AD,LOW);
//          digitalWrite(AE,LOW);
//          digitalWrite(AB,LOW);
        }
      }
      else{
//        Serial.println("Start the car");
        torque_left = 0;
        torque_right = 0;
//        digitalWrite(AD,HIGH);
//        digitalWrite(AE,LOW);
//        digitalWrite(AB,HIGH);
      }
    }
    else{
//      Serial.println("Do Precharge");
      torque_left = 0;
      torque_right = 0;
//      digitalWrite(AD,HIGH);
//      digitalWrite(AE,HIGH);
//      digitalWrite(AB,LOW);
    }
  }
  else{
//    Serial.println("Sensor Disconnected");
    torque_left = 0;
    torque_right = 0;
//    digitalWrite(AD,HIGH);
//    digitalWrite(AE,HIGH);
//    digitalWrite(AB,HIGH);
  }
  if((enable_receive_right == 1 || enable_receive_left == 1) && rtds){
    //Serial.print("\nSend torque \n");
    
    send_left();
    send_right();
    //send_torque_cmd = false;
  }
//  if((prechare()){
//    rtds = false;
//    torque_left_number = 0;
//    torque_right_number = 0;
//  }
//   Serial.println(bps1_in);
 // Serial.println(bps2_in);
  //Serial.println(apps2_in - apps1_in);
  Serial.print("Steer_in:");
  Serial.println(steer_in);
  Serial.print("Bps1:");
  Serial.println(bps1_in);
  Serial.print("Bps2:");
  Serial.println(bps2_in);
//  Serial.println(apps1_in);
  //Serial.println(apps1_normalised - apps2_normalised);
  // Serial.println(apps1_normalised);
//  Serial.println(apps2_in);
  Serial.println("hello");
//  if((apps1_normalised - apps2_normalised) > 0.1 || (apps1_in - apps2_in) == 0){
//    Serial.println(apps1_normalised - apps2_normalised);
//  }
  Serial.println(deration_factor);
//  Serial.println(rtds);
//  Serial.println(analogRead(c_plus));
//  Serial.println(voltage_right_receive);
//  Serial.println(voltage_left_receive);
  //Serial.println(enable_receive_left);
  //Serial.println(enable_receive_right);
//  Serial.println(fixed_rpm_max);
//  digitalRead(digitalRead(dr_select));

//    Serial.println(analogRead(ignition_sig));
//    
//    Serial.println(rtds);


}
