#ifndef UART_CONTROL_H
#define UART_CONTROL_H

#include "drive.h"
#include "driver/mcpwm_prelude.h"

// Struct to pass the PID parameters for each motor to pass to UART Task
typedef struct{
    pid_controller_t *pid_motors_m1;
    pid_controller_t *pid_motors_m2;
    taskCommand_t *motor_speed_task_command;
    mcpwm_cmpr_handle_t *motor1_comparator;
    mcpwm_cmpr_handle_t *motor2_comparator;

}uart_control_input_t;


void init_uart();
void uart_control_task(void *arg);

#endif
