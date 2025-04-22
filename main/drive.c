#include "drive.h"

// Helper function to initialize pid controllers
void initialize_pid_controller(pid_controller_t *pid_params, float target_value, float kp, float ki, float kd, float integral_limit_max, float integral_limit_min){
    pid_params->target_value = target_value;
    pid_params->kp = kp;
    pid_params->ki = ki;
    pid_params->kd = kd;
    pid_params->integral_limit_max = integral_limit_max;
    pid_params->integral_limit_min = integral_limit_min;
};