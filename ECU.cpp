#include "ECU.h"
// #include <Arduino.h>

void setup_can() {
    c_can.begin();
    c_can.setBaudRate(500000);
    c_can.enableFIFO();
    c_can.enableFIFOInterrupt();
    c_can.onReceive(mc_read);

    // d_can setup
    d_can.begin();
    d_can.setBaudRate(1000000);
    d_can.enableFIFO();
    d_can.enableFIFOInterrupt();
    // d_can.onReceive(can_receive);
}

void calc_norm() {
    APPS1.norm = (float)(APPS1.clip - APPS1.min) / (APPS1.max - APPS1.min);
    APPS2.norm = (float)(APPS2.clip - APPS2.min) / (APPS2.max - APPS2.min);
    // BPS1.norm = (float)(BPS1.clip - BPS1.min) / (BPS1.max - BPS1.min);
    // BPS2.norm = (float)(BPS2.clip - BPS2.min) / (BPS2.max - BPS2.min);
}

void clip_values() {
    APPS1.clip = constrain(APPS1.raw, APPS1.min, APPS1.max);
    APPS2.clip = constrain(APPS2.raw, APPS2.min, APPS2.max);
    // BPS1.clip = constrain(BPS1.raw, BPS1.min, BPS1.max);
    // BPS2.clip = constrain(BPS2.raw, BPS2.min, BPS2.max);
}

bool apps_range_error() {
    if(APPS1.raw > APPS1.max || APPS1.raw < APPS1.min){
        return true;
    }
    if(APPS2.raw > APPS2.max || APPS2.raw < APPS2.min){
        return true;
    }
    return false;
}

bool bps_range_error() {
    if(BPS1.raw > BPS1.max || BPS1.raw < BPS1.min){
        return true;
    }
    if(BPS2.raw > BPS2.max || BPS2.raw < BPS2.min){
        return true;
    }
    return false;
}

bool apps_10percent_error() {
    double apps_diff = abs(APPS1.norm - APPS2.norm);
    if (apps_diff > APPS_IMPLAUSIBILITY_PERCENTAGE) {
        if (APPS_10percent_timer == 0) {
            APPS_10percent_timer = millis();
        }
        else if (millis() - APPS_10percent_timer > APPS_IMPLAUSIBILITY_TIME) {
            return true;
        }
    }
    return false;
}

// bool bps_implausibility_error() {
//     double bps_diff = abs(BPS1.norm - BPS2.norm);
//     return (bps_diff > BPS_IMPLAUSIBILITY_PERCENTAGE);
// }

bool apps_equality_error() {
    if (APPS1.norm == APPS2.norm) {
        if (APPS_equality_timer == 0)
            APPS_equality_timer = millis();
        else if (millis() - APPS_equality_timer > APPS_EQUAL_TIME) {
            return true;
        }
    }
    return false;
}

bool brake_pressed() {
    if(BPS1.norm > BRAKE_LIGHT_THRESHOLD) {
        return true;
    }
    //Brake light only depends on BPS1, BPS2 is not used for brake light.
    // else if(BPS2.norm > BRAKE_LIGHT_THRESHOLD){
    //     return true;
    // }
    else if(bps_range_error()){
        return true;
    }
    return false;
}

void brake_light() {
    if(brake_pressed()){
        digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    }
    else{
        digitalWrite(BRAKE_LIGHT_PIN, LOW);
    }
}

double pressure_mapper(double pressure) {
    double p = ((pressure*4.0)/138 + 0.5)*(1024.0/5);
    return p;
}

void handle_precharge() {
    if (digitalRead(CPLUS_PIN) == LOW){// reverse logic
        on();
        precharged = true;
    }
    else{
        off();
        rtds = false;
        precharged = false;
    }
    precharged = false;
}

void start_up() {
    if (!rtds && (digitalRead(IGNITION_PIN) == LOW) && brake_pressed()) {
        digitalWrite(RTDS_PIN, HIGH);   //turns on Light and the Beep representing ready to drive state
        delay(2000);                    //the beep and light will be on for 2 seconds
        digitalWrite(RTDS_PIN, LOW);
        rtds = true;
    }
}

//Need to implement emi protection, determine better way to set torque threshold
void on() {
    digitalWrite(RFE_PIN, HIGH);
    delay(500);     //Earlier it was 0.5ms(mostly mistaken to be 0.5s), but datasheet specifies 500ms
    digitalWrite(FRG_PIN, HIGH);
}

void off() {
    digitalWrite(FRG_PIN, LOW);
    delay(1000);
    digitalWrite(RFE_PIN, LOW);
}


void send_left() {
    // Serial.print("mc_left torque: "); Serial.println(mc_left.torque);
    mc_left.torque = - mc_left.torque;
    torque_send_left[2] = mc_left.torque >> 8;
    torque_send_left[1] = mc_left.torque & 255;

    for ( uint8_t i = 0; i < 8; i++ )
        msg.buf[i] = torque_send_left[i];
    
    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);
}

void send_right() {
    mc_right.torque = mc_right.torque;
    // Serial.print("mc_right torque "); Serial.println(mc_right.torque);
    torque_send_right[2] = mc_right.torque >> 8;
    torque_send_right[1] = mc_right.torque & 255;

    for(uint8_t i = 0; i < 8; i++)
        msg.buf[i] = torque_send_right[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);
}

void mc_read(const CAN_message_t &msg) {
    if(msg.id == APPS_ID) {
        APPS1.raw = ((msg.buf[1] & 3) << 8) + msg.buf[0];
        APPS2.raw = ((msg.buf[2] & 15) << 6) + (msg.buf[1] >> 2);
        BPS1.raw = ((msg.buf[3] & 63) << 4) + (msg.buf[2] >> 4);
        BPS2.raw = (msg.buf[4] << 2) + (msg.buf[3] >> 6);
    }
    else if(msg.id == MC_LEFT_TRANSMIT) {
        // Handle left motor controller data
        switch(msg.buf[0]) {
            case 0x30: 
                mc_left.rpm = -(((msg.buf[2]<<8) | msg.buf[1])); break;
            case 0xE8: 
                mc_left.enable = msg.buf[1]; break;
            case 0xA0: 
                mc_left.torque = -(((msg.buf[2]<<8) | msg.buf[1])); break;
            case 0xEB: 
                mc_left.voltage = ((msg.buf[2]<<8) | msg.buf[1]); break;
            case 0xF6: 
                mc_left.power = ((msg.buf[2]<<8) | msg.buf[1]); break;
            case 0x20: 
                mc_left.current = ((msg.buf[2]<<8) | msg.buf[1]); break;
            case 0x49: 
                mc_left.motor_temp = ((msg.buf[2]<<8) | msg.buf[1]); break;
            case 0x4A: 
                mc_left.controller_temp = ((msg.buf[2]<<8) | msg.buf[1]); break;
        }
    }
    else if(msg.id == MC_RIGHT_TRANSMIT) {
        // Handle right motor controller data
        switch(msg.buf[0]) {
            case 0x30: 
                mc_right.rpm = ((msg.buf[2]<<8) | msg.buf[1]); break;
            case 0xE8: 
                mc_right.enable = msg.buf[1]; break;
            case 0xA0: 
                mc_right.torque = ((msg.buf[2]<<8) | msg.buf[1]); 
                break;
            case 0xEB: 
                mc_right.voltage = ((msg.buf[2]<<8) | msg.buf[1]); 
                break;
            case 0xF6: 
                mc_right.power = ((msg.buf[2]<<8) | msg.buf[1]); 
                break;
            case 0x20: 
                mc_right.current = ((msg.buf[2]<<8) | msg.buf[1]); 
                break;
            case 0x49: 
                mc_right.motor_temp = ((msg.buf[2]<<8) | msg.buf[1]); 
                break;
            case 0x4A: 
                mc_right.controller_temp = ((msg.buf[2]<<8) | msg.buf[1]); 
                break;
        }
    }
}

void initial_setup() {
    // // Disable send
    // for (uint8_t i = 0; i < 8; i++)
    //     msg.buf[i] = disable_send[i];

    // msg.id = MC_RIGHT_RECEIVE;
    // c_can.write(msg);

    // msg.id = MC_LEFT_RECEIVE;
    // c_can.write(msg);

    // Enable request
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = enable_request[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);

    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);

    // Speed request
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = speed_request[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);

    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);

    // Torque request
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = torque_request[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);

    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);

    // Voltage request
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = voltage_request[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);

    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);

    // Power request
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = power_request[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);

    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);

    // Current request
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = current_request[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);

    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);

    // Motor temperature request
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = motor_temp_request[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);

    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);

    // Controller temperature request
    for (uint8_t i = 0; i < 8; i++)
        msg.buf[i] = mc_temp_request[i];

    msg.id = MC_RIGHT_RECEIVE;
    c_can.write(msg);

    msg.id = MC_LEFT_RECEIVE;
    c_can.write(msg);
}
