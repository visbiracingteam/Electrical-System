#ifndef ECU_H
#define ECU_H

#include <Arduino.h>
#include <FlexCAN_T4.h>


// CAN IDs
#define APPS_ID 0x104
#define AMS_ID 0x522
#define RPM_ID 0x78
#define YAW_ID 0x70
#define DASH_SENT_ID 0xA9
#define MC_LEFT_RECEIVE 0x202
#define MC_RIGHT_RECEIVE 0x201
#define MC_LEFT_TRANSMIT 0x182
#define MC_RIGHT_TRANSMIT 0x181

// Pin Mappings
#define RTDS_PIN 4
#define BRAKE_LIGHT_PIN 2
#define IGNITION_PIN 20
#define DRIVER_MODE_PIN 21
#define ECU_ERROR_PIN 3
#define CPLUS_PIN 18
#define FRG_PIN 6
#define RFE_PIN 7

//Function Declarations

// CAN communication
void setup_can();
void mc_read(const CAN_message_t &msg);

// Sensor data handling
struct SensorData {
    uint16_t raw;
    double norm;
    uint16_t clip;
    uint16_t max;
    uint16_t min;
};

void calc_norm();
void clip_values();

// Error checking
bool apps_range_error();
bool bps_range_error();
bool apps_10percent_error();
// bool bps_implausibility_error();
bool apps_equality_error();

// Motor controller data
struct MotorControllerData {
    bool enable;
    int rpm;
    int torque;
    double voltage;
    double current;
    double power;
    int motor_temp;
    int controller_temp;
};

// System control
void handle_precharge();
void start_up();
void on();
void off();

// Motor controller communication
void send_left();
void send_right();
void initial_setup();
void set_avg_torque();

// Other utility functions
bool brake_pressed();
void brake_light();
double pressure_mapper(double pressure);

//Variables

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> c_can;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> d_can;
CAN_message_t msg;

SensorData APPS1, APPS2, BPS1, BPS2;
MotorControllerData mc_left, mc_right;

bool precharged = false;

// Clipping Values
const uint16_t APPS_GMAX = 1000;
const uint16_t APPS_GMIN = 10;
const uint16_t BPS_GMAX = 925;
const uint16_t BPS_GMIN = 98;

// Torque calculation variables
// const double ecu_max_torque;  // Nm
const double ndrive_max_torque = 90;  // Nm
double torque_avg = 0;

bool rtds = false;

// NDrive Parameters
const int fixed_Torquemax = ndrive_max_torque;
int fixed_rpm_max = 32767;
bool change_torque_status = true;
int change_torque_timer = 0;

// Implausibility Thresholds
const float APPS_IMPLAUSIBILITY_PERCENTAGE = 0.1;
const float BPS_IMPLAUSIBILITY_PERCENTAGE = 0.1;

// Brake Light Threshold
const uint16_t bps_thr = 15; //bar
uint16_t BRAKE_LIGHT_THRESHOLD = 150;

// Timers
const unsigned long APPS_IMPLAUSIBILITY_TIME = 100;  // 100ms
const unsigned long APPS_EQUAL_TIME = 100;           // 100ms
// const unsigned long BPS_IMPLAUSIBILITY_TIME = 100;   // 100ms
double APPS_10percent_timer = 0, APPS_equality_timer = 0;

/* MC enable Flags */
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
// Add other necessary private variables and methods
// };

#endif;
