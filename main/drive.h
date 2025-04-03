#ifndef DRIVE_H
#define DRIVE_H

/* PCNT and MCPWM Configurations */

#define ENCODER1_PIN_A GPIO_NUM_36
#define ENCODER1_PIN_B GPIO_NUM_39
#define ENCODER2_PIN_A GPIO_NUM_35
#define ENCODER2_PIN_B GPIO_NUM_37 // Use GPIO_NUM_34 for ESP32 (actual setup), 37 is for ESP32S3

#define PCNT_HIGH_LIMIT 32767 // Max for signed 16 bit counter
#define PCNT_LOW_LIMIT -32768 // Min for signed 16 bit counter

#define MOTOR_1_DIR GPIO_NUM_26
#define MOTOR_2_DIR GPIO_NUM_33

#define MOTOR_1_PWM GPIO_NUM_21 // Under timer0, Use GPIO_NUM_25 for ESP32 (actual setup), 21 is for ESP32S3
#define MOTOR_2_PWM GPIO_NUM_47// Under timer0, Use GPIO_NUM_32 for ESP32 (actual setup), 47 is for ESP32S3

// PWM Cofiguration
#define TIMER_RESOLUTION 80000000 // 80Mhz which is half of the 160Mhz source used
#define COUNTER_PERIOD 4000 // 4000 ticks for 20kHz PWM used for Cytron MD10C motor driver

void pcnt_init();
void mcpwm_init();

#endif