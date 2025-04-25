#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

// Button Input GPIO
#define BUTTON_PIN GPIO_NUM_15

// GPOI
#define COLOR_SENSOR_LED GPIO_NUM_5

// LED GPIO
#define LED1 GPIO_NUM_14
#define LED2 GPIO_NUM_13
#define LED3 GPIO_NUM_27

// VL53 Numbers
#define VL53_FRONT_LEFT 3
#define VL53_FRONT_RIGHT 2
#define VL53_FRONT_CENTER 0
#define VL53_LEFT 4
#define VL53z_RIGHT 1

/* PCNT and MCPWM Configurations */
#define ENCODER2_PIN_A GPIO_NUM_36
#define ENCODER2_PIN_B GPIO_NUM_39
#define ENCODER1_PIN_A GPIO_NUM_34
#define ENCODER1_PIN_B GPIO_NUM_35 // Use GPIO_NUM_34 for ESP32 (actual setup), 37 is for ESP32S3

#define PCNT_HIGH_LIMIT 32767 // Max for signed 16 bit counter
#define PCNT_LOW_LIMIT -32768 // Min for signed 16 bit counter

#define MOTOR_1_DIR GPIO_NUM_26
#define MOTOR_2_DIR GPIO_NUM_33

#define MOTOR_1_PWM GPIO_NUM_25 // Under timer0, Use GPIO_NUM_25 for ESP32 (actual setup), 21 is for ESP32S3
#define MOTOR_2_PWM GPIO_NUM_32// Under timer0, Use GPIO_NUM_32 for ESP32 (actual setup), 47 is for ESP32S3

// PWM Cofiguration
#define TIMER_RESOLUTION 80000000 // 80Mhz which is half of the 160Mhz source used
#define COUNTER_PERIOD 4000 // 4000 ticks for 20kHz PWM used for Cytron MD10C motor driver
#define MCPWM_COMPARATOR_MAX 1999

// Servo PWM Configuration
#define SERVO_PWM_PIN GPIO_NUM_2
#define SERVO_PWM_RESOLUTION 1600000
#define SERVO_COUNTER_PERIOD 32000 // 50Hz PWM for servo

#define SERVO_ANGLE_TO_COMPARATOR(angle) (((angle* 3200) / 180  )+580) // 0 to 180 degrees


// Encoder
#define REDUCER_GEAR_RATIO 20.409
#define ENCODER_PPR 13

typedef enum {
    RUN,
    STOP,
    RESTART,
} taskCommand_t;

// used for deciding the type of movement
typedef enum {
    TURN,
    STRAIGHT,
    DO_NOT_MOVE,
    MANUAL,
    WALL_CALIBRATION,
} robot_move_t;

typedef enum {
    EMPTY,
    WALL,
    NOT_VALID,
} vl53_detection_t;

// MPU6050 movement control PID parametes
// THese parametes are tuned for 8ms interval
// Output unit of the PID controller is motor speed in pulse per second
#define PID_STRAIGHT_KP 0.07
#define PID_STRAIGHT_KI 0.00001
#define PID_STRAIGHT_KD 0
#define PID_STRAIGHT_MAX_INTEGRAL 0.5
#define PID_STRAIGHT_MIN_INTEGRAL -0.5
#define PID_STRAIGHT_TOLERANCE 1

// MPU6050 Turning PID Parameters
#define PID_TURN_KP 0.045
#define PID_TURN_KI 0.000005
#define PID_TURN_KD 0
#define PID_TURN_MAX_INTEGRAL 0.5
#define PID_TURN_MIN_INTEGRAL -0.5
#define PID_TURN_TOLERANCE 1


// Motor speed control PID parameters
// THese parametes are tuned for 8ms interval
// Maximum pulse per rotation remains untested !!!
#define MOTOR_SPEED_KP 200
#define MOTOR_SPEED_KI 20
#define MOTOR_SPEED_KD 0
#define MOTOR_SPEED_MAX_INTEGRAL 1800
#define MOTOR_SPEED_MIN_INTEGRAL -1800
#define MOTOR_SPEED_PID_PERIOD 8 // 8ms

#define WALL_CALIBRATION_KP 0.02
#define WALL_CALIBRATION_KI 0.00002
#define WALL_CALIBRATION_KD 0
#define WALL_CALIBRATION_MAX_INTEGRAL 0.5
#define WALL_CALIBRATION_MIN_INTEGRAL -0.5
#define WALL_CALIBRATION_TOLERANCE 3

typedef struct {
    float target_value;
    float kp;
    float ki;
    float kd;
    float integral_limit_max;
    float integral_limit_min;
    // The items need not to be configured during initialization
    float integral;
    float prev_err;
    float pTerm;
    float iTerm; 
    float dTerm; 
    float curr_value; 
    float err; 
    float output;
}pid_controller_t;

typedef struct {
    int prev_count;
    int curr_count;
    int delta_count;
}encoder_pulses_t;


typedef struct {
    uint16_t distance;
    vl53_detection_t detection;
}vl53_distancce_t;

void pcnt_init();
void mcpwm_init();
void motor_pid_speed_control();
void mpu6050_move_straight_pid();
void set_motor_speed();
void initialize_pid_controller(pid_controller_t *pid_params, float target_value, float kp, float ki, float kd, float integral_limit_max, float integral_limit_min);
void mpu6050_turn_pid();
void wall_calibration_pid();
#endif