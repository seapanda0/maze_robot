#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_random.h"

#include <math.h>
#include <stdlib.h> // For abs()

#include "nvs.h"
#include "tcs34725.h"
#include "vlx53l0x.h"
#include "i2c_configuration.h"
#include "mpu6050.h"
#include "drive.h"
#include "uart_control.h"

#define MAIN_LOG_TAG "MAIN"

/* GPIO Configuration */
#define GPIO_BIT_MASK (1ULL << X_SHUT_1) | (1ULL << X_SHUT_2) | (1ULL << X_SHUT_3) | (1ULL << X_SHUT_4) \
    | (1ULL << MOTOR_1_DIR) | (1ULL << MOTOR_2_DIR) | (1ULL << COLOR_SENSOR_LED) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3)

/* I2C Device Handlers */
i2c_master_dev_handle_t tcs34725_handle = NULL, tca95_handle = NULL;
i2c_master_dev_handle_t mpu6050_handle = NULL;
i2c_master_dev_handle_t vl53l0x_default, vl53l0x_sensor0, vl53l0x_sensor1, vl53l0x_sensor2, vl53l0x_sensor3, vl53l0x_sensor4;
i2c_master_dev_handle_t *vl53l0x_arr[5] = {&vl53l0x_sensor0, &vl53l0x_sensor1, &vl53l0x_sensor2, &vl53l0x_sensor3, &vl53l0x_sensor4};

/* PCNT Handlers */
pcnt_unit_handle_t encoder1 = NULL, encoder2 = NULL;

/* MCPWM Comparator Handlers */
mcpwm_cmpr_handle_t motor1 = NULL, motor2 = NULL;
mcpwm_cmpr_handle_t servo = NULL;

/* FreeRTOS Task Handlers */
TaskHandle_t sensor_sampling_handler = NULL, move_straight_task_handler = NULL;
TaskHandle_t wall_detection_task_handler = NULL;
TaskHandle_t calibrate_color_sensor_task_handler = NULL;
TaskHandle_t uart_control_task_handler = NULL;
TaskHandle_t color_sensor_compute_task_handler = NULL;
TaskHandle_t maze_logic_task_handler = NULL;

/* Task command status */
taskCommand_t motor_speed_task_command = STOP; 
taskCommand_t mpu6050_move_straight_command = STOP; 
taskCommand_t mpu6050_turn_command = STOP; 
taskCommand_t wall_calibration_turn_command = STOP; 
robot_move_t robot_move_type = DO_NOT_MOVE;

/* Variables measuring timing */
int64_t prev_time = 0;
int64_t curr_time = 0;
int64_t delta_time = 0;

/*Timer Variables*/
uint32_t cycle_count = 0;

/*Sensor Readings*/
float angleZ = 0;
uint32_t r[3] = {0}, g[3] = {0}, b[3] = {0}, c[3] = {0};
float normalized_data[3][3];
vl53_distancce_t vl53[5] = {0};
int encoder_1_count = 0, encoder_2_count = 0;
float global_x = 0; 
float delta_x = 0;
float v_left;
float v_right;

/*PID Structs*/
pid_controller_t pid_motors[2] = {0};
pid_controller_t mpu6050_pid_params = {0};
encoder_pulses_t encoder_pulses[2] = {0};
pid_controller_t mpu6050_turn_pid_params = {0};
pid_controller_t wall_calibration_pid_params = {0};
bool mpu6050_turn_in_position = false;
bool wall_calibration_in_position = false;

/*Motor controls*/
float basespeed_m0 = 1.25;
float basespeed_m1 = 1.25;
int servo_pos = 0;

/*Manual Overrides Via UART*/
char keyboard_input = 0;
uart_control_input_t uart_task_args = {
    .pid_motors_m1 = &pid_motors[0],
    .pid_motors_m2 = &pid_motors[1],
    .motor_speed_task_command = &motor_speed_task_command,
    .robot_move_type = &robot_move_type,
    .motor1_comparator = &motor1,
    .motor2_comparator = &motor2,
};

/*Color calibration state*/
bool calibrate_color = false;
float normalized_calibration[7][3];
float eucd_distances_left[3];
float eucd_distances_right[3];
float eucd_distances_bottom[4];
bool left_color_detected = false;
bool right_color_detected = false;
bool bottom_color_detected = false;

/*Maze Logic FreeRTOS Command codes*/
maze_event_t maze_event = WALL_EVENT;
bool reset_kinematiccs = false;
bool fire_wall_event = false;
bool fire_side_wall_event = false;

robot_state_t robot_state = ROBOT_MAKE_DECISION;

void servo_blink_extrude(void *args);

bool sensors_sampling_isr_handler (gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){
    BaseType_t xTaskawaken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_sampling_handler, &xTaskawaken);

    // vTaskNotifyGiveFromISR(move_straight_task_handler, &xTaskawaken);
    cycle_count++;
    return xTaskawaken;
}

// VL53L0X Periodic Task, wakes up every 35ms
// vl53l0x_isr_handler will wake up this task 
void sensors_sampling_task (void *arg){
    uint8_t write_buf = RESULT_RANGE_STATUS + 10; // assumptions: Linearity Corrective Gain is 1000 (default)
    uint8_t write_buf_clear[2] = {SYSTEM_INTERRUPT_CLEAR, 0x01};
    uint8_t read_buf[2] = {0};
    esp_err_t status = ESP_OK;
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // prev_time = esp_timer_get_time();
        // delta_time = curr_time - prev_time;
        
        // Every cycle, 4ms, sample mpu6050
        mpu6050_updateZ(&mpu6050_handle , &angleZ);

        // Every 4ms calculate kinematics
        kinematics();

        // Every 2 cycle , 8ms, sample wheel encoder and perform PID control
        if (cycle_count % 2 == 0)
        {
            switch (robot_move_type)
            {
            case STRAIGHT:{
                mpu6050_move_straight_pid();
                pid_motors[0].target_value = basespeed_m0 + mpu6050_pid_params.output;
                pid_motors[1].target_value = basespeed_m0 - mpu6050_pid_params.output;
                break;
            }
            case REVERSE:{
                mpu6050_move_straight_pid();
                pid_motors[0].target_value = -basespeed_m0 + mpu6050_pid_params.output;
                pid_motors[1].target_value = -basespeed_m0 - mpu6050_pid_params.output;
                break;
            }
            case TURN:{
                mpu6050_turn_pid();
                // ESP_LOGI(MAIN_LOG_TAG, "Turn PID Output: %f", mpu6050_turn_pid_params.output);
                if (mpu6050_turn_pid_params.output > 0)
                {
                    pid_motors[0].target_value = mpu6050_turn_pid_params.output;
                    pid_motors[1].target_value = 0;
                }
                else if (mpu6050_turn_pid_params.output < 0)
                {
                    pid_motors[0].target_value = 0;
                    pid_motors[1].target_value = -mpu6050_turn_pid_params.output;
                }
                break;
            }case DO_NOT_MOVE:{
                pid_motors[0].target_value = 0;
                pid_motors[1].target_value = 0;
                break;
            }case MANUAL:{
                break;
            }case WALL_CALIBRATION:{
                wall_calibration_pid();
                pid_motors[0].target_value = wall_calibration_pid_params.output;
                pid_motors[1].target_value = -wall_calibration_pid_params.output;


            }
        }
            pcnt_unit_get_count(encoder1, &encoder_pulses[0].curr_count);
            pcnt_unit_get_count(encoder2, &encoder_pulses[1].curr_count);
            motor_pid_speed_control();
        }
        
        // Every 9th cycle, 36ms, sample vl53l0x
        if (cycle_count % 9 == 0){
            for (int i = 0; i < 5; i++){
                status |= i2c_master_transmit_receive(*vl53l0x_arr[i], &write_buf, 1, read_buf, 2, I2C_TIMEOUT_MS);
                status |= i2c_master_transmit(*vl53l0x_arr[i], write_buf_clear, 2 , I2C_TIMEOUT_MS);
                if (status == ESP_OK) {        
                    vl53[i].distance = read_buf[1] | (read_buf[0] << 8);
                } // Implement error handling!!!
            }
            // we wont see vl53 over 800 in our application anyway
            if (vl53[VL53_FRONT_CENTER].distance > 100)
            {
                vl53[VL53_FRONT_CENTER].detection = EMPTY;
            }
            else{
                vl53[VL53_FRONT_CENTER].detection = WALL;
            }

            if (vl53[VL53_LEFT].distance > 200)
            {
                vl53[VL53_LEFT].detection = EMPTY;
            }
            else{
                vl53[VL53_LEFT].detection = WALL;
            }

            if (vl53[VL53z_RIGHT].distance > 200)
            {
                vl53[VL53z_RIGHT].detection = EMPTY;
            }
            else{
                vl53[VL53z_RIGHT].detection = WALL;
            }
            // xTaskNotifyGive(wall_detection_task_handler);
            
        }
        
        // Every 60th cycle, 240ms, sample tcs34725
        if (cycle_count % 60 == 0){
            tcs34725_read_raw_multi(&tcs34725_handle, &tca95_handle, c, r, g, b);  // Implement error handling!!!
            for (int  i = 0; i < 3; i++){
                normalizeRGB((float)r[i], (float)g[i], (float)b[i], (float)c[i], normalized_data[i]);
            }
            xTaskNotifyGive(color_sensor_compute_task_handler);

            if (calibrate_color){
                xTaskNotifyGive(calibrate_color_sensor_task_handler);
            }

        }
        // prev_time = curr_time;
        // curr_time = esp_timer_get_time();
        // delta_time = curr_time - prev_time;
        // if (delta_time > 3000){
            // ESP_LOGI(MAIN_LOG_TAG, "Time taken: %lld us", delta_time);
        // }
    }
}

void color_sensor_compute_task (void * arg){
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait until new sensor data is avaliable
        // prev_time = esp_timer_get_time();

        // compute left sensor
        for(int i = 0; i < 3; i++){
            eucd_distances_left[i] = colorDistance(normalized_data[COLOR_SENSOR_LEFT][0], normalized_data[COLOR_SENSOR_LEFT][1], normalized_data[COLOR_SENSOR_LEFT][2], 
                normalized_calibration[i][0], normalized_calibration[i][1], normalized_calibration[i][2]);
            }
            // ESP_LOGI(TCS34725_LOG_TAG, " Left Sensor: Green %f Blue %f Black %f",  eucd_distances_left[0], eucd_distances_left[1], eucd_distances_left[2]);
            
        // compute right sensor
        for(int i = 0; i < 3; i++){
            eucd_distances_right[i] = colorDistance(normalized_data[COLOR_SENSOR_RIGHT][0], normalized_data[COLOR_SENSOR_RIGHT][1], normalized_data[COLOR_SENSOR_RIGHT][2], 
                normalized_calibration[i][0], normalized_calibration[i][1], normalized_calibration[i][2]);
            }
            // ESP_LOGI(TCS34725_LOG_TAG, " Right Sensor: Green %f Blue %f Black %f",  eucd_distances_right[0], eucd_distances_right[1], eucd_distances_right[2]);

        // compute bottom sensor
        for(int i = 0; i < 4; i++){
            eucd_distances_bottom[i] = colorDistance(normalized_data[COLOR_SENSOR_CENTER][0], normalized_data[COLOR_SENSOR_CENTER][1], normalized_data[COLOR_SENSOR_CENTER][2], 
                normalized_calibration[3 +i][0], normalized_calibration[3+i][1], normalized_calibration[3+i][2]);
            }
            // ESP_LOGI(TCS34725_LOG_TAG, " Bottom Sensor: Black %f Red %f Blue %f Yellow %f",  eucd_distances_bottom[0], eucd_distances_bottom[1], eucd_distances_bottom[2], eucd_distances_bottom[4]);
        
        left_color_detected = false;
        right_color_detected = false;
        bottom_color_detected = false;
        for(int i = 0; i < 3; i++){
            if (eucd_distances_left[i] < FIRING_THRESHOLD){
                left_color_detected = true;
                ESP_LOGI(TCS34725_LOG_TAG, "Left detected color %d", i);
                xTaskCreatePinnedToCore(servo_blink_extrude, "servo", 2048, NULL, 15, NULL, 1);

            }
            if (eucd_distances_right[i] < FIRING_THRESHOLD){
                right_color_detected = true;
                ESP_LOGI(TCS34725_LOG_TAG, "Right detected color %d", i);
                xTaskCreatePinnedToCore(servo_blink_extrude, "servo", 2048, NULL, 15, NULL, 1);

            }

        }
        for(int i = 0; i < 4; i++){
            if (eucd_distances_bottom[i] < FIRING_THRESHOLD){
                bottom_color_detected = true;
                ESP_LOGI(TCS34725_LOG_TAG, "Bottom detected color %d", i);

            }
        }
        // curr_time = esp_timer_get_time();

        
        // ESP_LOGI(TCS34725_LOG_TAG, "Distance %lld", curr_time-prev_time);
    }

}

void kinematics(){
    // Every 4ms calculate speed once to for kinematic
    if (reset_kinematiccs == true){
        global_x = 0;
    }

    pcnt_unit_get_count(encoder1, &encoder_pulses[0].curr_count_kine);
    pcnt_unit_get_count(encoder2, &encoder_pulses[1].curr_count_kine);
    for (int i = 0; i < 2; i++){
        encoder_pulses[i].delta_count_kine = (encoder_pulses[i].curr_count_kine - encoder_pulses[i].prev_count_kine);
        encoder_pulses[i].prev_count_kine = encoder_pulses[i].curr_count_kine;
    }
    // Current units are now in cm per mili second
    v_left = ((float)encoder_pulses[0].delta_count_kine / ( PULSE_PER_ROTATION)) * (2 * PI * WHEEL_RADIUS);
    v_right = ((float)encoder_pulses[1].delta_count_kine / ( PULSE_PER_ROTATION)) * (2 * PI * WHEEL_RADIUS);
    
    // Units are in cm
    delta_x = ((v_left + v_right)/2);
    global_x += delta_x;
    
    // prev_time = curr_time;
    // curr_time = esp_timer_get_time();
    // delta_time = curr_time - prev_time;

}

void color_calibration_task(void *arg){
    
    float temp_r;
    float temp_g;
    float temp_b;
    float temp_c;

    ESP_LOGI(TCS34725_LOG_TAG, "Starting calibrations, start with Green on Left Sensor");
    // For side sensor
    for (int i = 0; i < 3; i ++){
        temp_r = 0;
        temp_g = 0;
        temp_b = 0;
        temp_c = 0;

        ESP_LOGI(TCS34725_LOG_TAG, "Ready to calibrate color code K%d on LEFT SENSOR. Press any key to start....", i);
        wait_for_character();
        ESP_LOGI(TCS34725_LOG_TAG, "Calibrating, DO NOT MOVE THE ROBOT");
        for (int j = 0; j <= TCS34725_SAMPLE_SIZE; j++){
            // the sensor sampling task will fire notification based on this variable
            calibrate_color = true;
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait until new sensor data is avaliable
            temp_r += r[COLOR_SENSOR_LEFT];
            temp_g += g[COLOR_SENSOR_LEFT];
            temp_b += b[COLOR_SENSOR_LEFT];
            temp_c += c[COLOR_SENSOR_LEFT];
        }
        calibrate_color = false;
        temp_r = temp_r / TCS34725_SAMPLE_SIZE;
        temp_g = temp_g / TCS34725_SAMPLE_SIZE;
        temp_b = temp_b / TCS34725_SAMPLE_SIZE;
        temp_c = temp_c / TCS34725_SAMPLE_SIZE;
        normalizeRGB(temp_r, temp_g, temp_b, temp_c, normalized_calibration[i]);
        ESP_LOGI(TCS34725_LOG_TAG, "Avg_R:%f, Avg_G:%f, Avg_B:%f, Avg_C:%f, NR:%f, NG:%f, NB:%f", temp_r, temp_g, temp_b, temp_c, normalized_calibration[i][0], normalized_calibration[i][1], normalized_calibration[i][2]);
        ESP_LOGI(TCS34725_LOG_TAG, "Calibration for color K%d Complete! \n", i);
    }
    ESP_LOGI(TCS34725_LOG_TAG, "SWITCHING TO BOTTOM SENSOR");
    for (int i = 3; i < 7; i ++){
        temp_r = 0;
        temp_g = 0;
        temp_b = 0;
        temp_c = 0;

        ESP_LOGI(TCS34725_LOG_TAG, "Ready to calibrate color code K%d on BOTTOM SENSOR. Press any key to start....", i);
        wait_for_character();
        ESP_LOGI(TCS34725_LOG_TAG, "Calibrating, DO NOT MOVE THE ROBOT");
        for (int j = 0; j <= TCS34725_SAMPLE_SIZE; j++){
            // the sensor sampling task will fire notification based on this variable
            calibrate_color = true;
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait until new sensor data is avaliable
            temp_r += r[COLOR_SENSOR_CENTER];
            temp_g += g[COLOR_SENSOR_CENTER];
            temp_b += b[COLOR_SENSOR_CENTER];
            temp_c += c[COLOR_SENSOR_CENTER];
        }
        calibrate_color = false;
        temp_r = temp_r / TCS34725_SAMPLE_SIZE;
        temp_g = temp_g / TCS34725_SAMPLE_SIZE;
        temp_b = temp_b / TCS34725_SAMPLE_SIZE;
        temp_c = temp_c / TCS34725_SAMPLE_SIZE;
        normalizeRGB(temp_r, temp_g, temp_b, temp_c, normalized_calibration[i]);
        ESP_LOGI(TCS34725_LOG_TAG, "Avg_R:%f, Avg_G:%f, Avg_B:%f, Avg_C:%f, NR:%f, NG:%f, NB:%f", temp_r, temp_g, temp_b, temp_c, normalized_calibration[i][0], normalized_calibration[i][1], normalized_calibration[i][2]);
        ESP_LOGI(TCS34725_LOG_TAG, "Calibration for color K%d Complete! \n", i);
    }

    ESP_LOGI(TCS34725_LOG_TAG, "Starting saving calibration data to nvs...");
    nvs_write(normalized_calibration, sizeof(normalized_calibration));
    ESP_LOGI(TCS34725_LOG_TAG, "Successful");
    nvs_flash_deinit();

    print_normalized(normalized_calibration);

    xTaskNotifyGive(uart_control_task_handler);
    vTaskDelete(calibrate_color_sensor_task_handler);
}

void wall_detection_task(void *arg){
    while(1){
        // Wait for the task to be notified
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Perform wall detection logic here
        // For example, check the distance from the sensors and take action
        // for (int i = 0; i < 5; i++){
        //     if (vl53.distance[i] < 1000){ // Example threshold
        //         ESP_LOGI(MAIN_LOG_TAG, "Wall detected by sensor %d", i);
        //     }
        // }
    }
}

void set_motor_power(int motor1_power, int motor2_power){
    uint32_t dir1 = 0, dir2 = 0;

    if (motor1_power > 0){
        dir1 = 1;
    }else{
        dir1 = 0;
    }

    if (motor2_power > 0){
        dir2 = 1;
    }else{ 
        dir2 = 0;
    }

    mcpwm_comparator_set_compare_value(motor1, abs(motor1_power));
    mcpwm_comparator_set_compare_value(motor2, abs(motor2_power));
    gpio_set_level(MOTOR_1_DIR, dir1);
    gpio_set_level(MOTOR_2_DIR, dir2);
}

void set_motor_speed(int motor1_speed, int motor2_speed){
    pid_motors[0].target_value = motor1_speed;
    pid_motors[1].target_value = motor2_speed;
}

void motor_pid_speed_control()
{
    switch (motor_speed_task_command){
    case RUN:
    {
        for (int i = 0; i < 2; i++)
        {
            encoder_pulses[i].delta_count = (encoder_pulses[i].curr_count - encoder_pulses[i].prev_count);
            encoder_pulses[i].prev_count = encoder_pulses[i].curr_count;

            // Calculate instantaneous velocity
            pid_motors[i].curr_value = (float)encoder_pulses[i].delta_count / (float)MOTOR_SPEED_PID_PERIOD;
            pid_motors[i].err = pid_motors[i].target_value - pid_motors[i].curr_value;

            pid_motors[i].pTerm = pid_motors[i].kp * pid_motors[i].err;

            // Calculate Integral term
            pid_motors[i].integral += pid_motors[i].err;
            pid_motors[i].iTerm = pid_motors[i].ki * pid_motors[i].integral;
            if (pid_motors[i].iTerm > pid_motors[i].integral_limit_max)
            {
                pid_motors[i].iTerm = pid_motors[i].integral_limit_max;
            }
            else if (pid_motors[i].iTerm < pid_motors[i].integral_limit_min)
            {
                pid_motors[i].iTerm = pid_motors[i].integral_limit_min;
            }

            // // Calculate Derivative term
            // pid_motors[i].dTerm = pid_motors[i].kd * (pid_motors[i].err - pid_motors[i].prev_err);
            // pid_motors[i].prev_err = pid_motors[i].err;

            pid_motors[i].output = (pid_motors[i].pTerm + pid_motors[i].iTerm + pid_motors[i].dTerm);

            // Limit the ouput to a range within valid comparator value
            if (pid_motors[i].output > MCPWM_COMPARATOR_MAX)
            {
                pid_motors[i].output = MCPWM_COMPARATOR_MAX;
            }
            else if (pid_motors[i].output < -MCPWM_COMPARATOR_MAX)
            {
                pid_motors[i].output = -MCPWM_COMPARATOR_MAX;
            }
        }
        set_motor_power(pid_motors[0].output, pid_motors[1].output);
        break;
    }
    case STOP:
    {
        break;
    }case RESTART:{
        motor_speed_task_command = RUN;
        break;
    }
    }
}

void mpu6050_move_straight_pid(){
    switch (mpu6050_move_straight_command){
        case RUN:{
            mpu6050_pid_params.err = mpu6050_pid_params.target_value - angleZ;

            // Calculate P term
            mpu6050_pid_params.pTerm = mpu6050_pid_params.kp * mpu6050_pid_params.err;
        
            // Calculate I term
            mpu6050_pid_params.integral += mpu6050_pid_params.err;
            mpu6050_pid_params.iTerm = mpu6050_pid_params.ki * mpu6050_pid_params.integral;
            if (mpu6050_pid_params.iTerm > mpu6050_pid_params.integral_limit_max)
            {
                mpu6050_pid_params.iTerm = mpu6050_pid_params.integral_limit_max;
            }
            else if (mpu6050_pid_params.iTerm < mpu6050_pid_params.integral_limit_min)
            {
                mpu6050_pid_params.iTerm = mpu6050_pid_params.integral_limit_min;
            }
        
            // Derivative not implemented
            
            // Calculate output and if it is within tolerance, set output to 0
            if (fabs(mpu6050_pid_params.err) < PID_STRAIGHT_TOLERANCE){
                mpu6050_pid_params.output = 0;
            } else {
                mpu6050_pid_params.output = mpu6050_pid_params.pTerm + mpu6050_pid_params.iTerm;
            }

            break;
            
        }case STOP:{
            mpu6050_pid_params.output = 0;
            mpu6050_pid_params.prev_err = 0;
            mpu6050_pid_params.target_value = 0;
            mpu6050_pid_params.integral = 0;
            break;
        }case RESTART:{
            // Every time the task is started, use current angleZ as target
            mpu6050_pid_params.target_value = angleZ;
            mpu6050_move_straight_command = RUN;
            break;
        }
    }
}

void mpu6050_turn_pid()
{
    static int in_position_cycle_count = 0;
    switch(mpu6050_turn_command){
        case RUN:{
            mpu6050_turn_pid_params.err = mpu6050_turn_pid_params.target_value - angleZ;

            // Calculate P term
            mpu6050_turn_pid_params.pTerm = mpu6050_turn_pid_params.kp * mpu6050_turn_pid_params.err;
        
            // Calculate I term
            mpu6050_turn_pid_params.integral += mpu6050_turn_pid_params.err;
            mpu6050_turn_pid_params.iTerm = mpu6050_turn_pid_params.ki * mpu6050_turn_pid_params.integral;
            if (mpu6050_turn_pid_params.iTerm > mpu6050_turn_pid_params.integral_limit_max)
            {
                mpu6050_turn_pid_params.iTerm = mpu6050_turn_pid_params.integral_limit_max;
            }
            else if (mpu6050_turn_pid_params.iTerm < mpu6050_turn_pid_params.integral_limit_min)
            {
                mpu6050_turn_pid_params.iTerm = mpu6050_turn_pid_params.integral_limit_min;
            }
        
            // Derivative not implemented
        
            // Calculate output and if it is within tolerance, set output to 0
            if (fabs(mpu6050_turn_pid_params.err) < PID_STRAIGHT_TOLERANCE)
            {
                in_position_cycle_count++;
                mpu6050_turn_pid_params.output = 0;
            }
            else{
                mpu6050_turn_pid_params.output = mpu6050_turn_pid_params.pTerm + mpu6050_turn_pid_params.iTerm;
            }
            if (in_position_cycle_count > 100){
                mpu6050_turn_pid_params.output = 0;
                mpu6050_turn_in_position = true;
                mpu6050_turn_command = STOP;
            }
        
            // if (mpu6050_turn_pid_params.output > 0){
            //     pid_motors[0].target_value = mpu6050_turn_pid_params.output;
            //     pid_motors[1].target_value = 0;
            // }else if (mpu6050_turn_pid_params.output < 0){
            //     pid_motors[0].target_value = 0;
            //     pid_motors[1].target_value = -mpu6050_turn_pid_params.output;
            // }
            break;
        }case STOP:{
            //clear pid parameters
            mpu6050_turn_in_position = false;
            in_position_cycle_count = 0;
            mpu6050_turn_pid_params.output = 0;
            mpu6050_turn_pid_params.integral = 0;
            mpu6050_turn_pid_params.prev_err = 0;
            mpu6050_turn_pid_params.curr_value = 0;
            break;

        }case RESTART:{
            // Angle Z need to be manually set before setting the command to restart
            mpu6050_turn_command = RUN;
            in_position_cycle_count = 0;
            mpu6050_turn_in_position = false;
            break;
        }
    }
    
}

void wall_calibration_pid(){
    static int in_position_cycle_count = 0;
    switch (wall_calibration_turn_command)
    {
    case RUN:
    {
        if (vl53[VL53_FRONT_LEFT].detection == NOT_VALID || vl53[VL53_FRONT_RIGHT].detection == NOT_VALID){
            wall_calibration_pid_params.output = 0;
            ESP_LOGE("VL53", "Sensor out of range");
        }
        else{
            wall_calibration_pid_params.err = vl53[VL53_FRONT_LEFT].distance - vl53[VL53_FRONT_RIGHT].distance;

            // Calculate P term
            wall_calibration_pid_params.pTerm = wall_calibration_pid_params.kp * wall_calibration_pid_params.err;

            // Calculate I term
            wall_calibration_pid_params.integral += wall_calibration_pid_params.err;
            wall_calibration_pid_params.iTerm = wall_calibration_pid_params.ki * wall_calibration_pid_params.integral;
            if (wall_calibration_pid_params.iTerm > wall_calibration_pid_params.integral_limit_max)
            {
                wall_calibration_pid_params.iTerm = wall_calibration_pid_params.integral_limit_max;
            }
            else if (wall_calibration_pid_params.iTerm < wall_calibration_pid_params.integral_limit_min)
            {
                wall_calibration_pid_params.iTerm = wall_calibration_pid_params.integral_limit_min;
            }

            // Derivative not implemented

            // Calculate output and if it is within tolerance, set output to 0
            if (fabs(wall_calibration_pid_params.err) < WALL_CALIBRATION_TOLERANCE)
            {
                in_position_cycle_count++;
                wall_calibration_pid_params.output = 0;
            }
            else
            {
                wall_calibration_pid_params.output = wall_calibration_pid_params.pTerm + wall_calibration_pid_params.iTerm;
            }
            // If robot is already facing wall straight, exit the wall calibration routine
            if (in_position_cycle_count > 100){
                wall_calibration_pid_params.output = 0;
                in_position_cycle_count = 0;
                wall_calibration_in_position = true;
                xTaskNotifyGiveIndexed(maze_logic_task_handler, WALL_CALIBRATION_SLOT);
                wall_calibration_turn_command = STOP;
            }
        }
        break;
    }
    case STOP:
    {
        // clear pid parameters

        break;
    }
    case RESTART:
    {
        in_position_cycle_count = 0;
        wall_calibration_pid_params.output = 0;
        wall_calibration_pid_params.integral = 0;
        wall_calibration_pid_params.prev_err = 0;
        wall_calibration_pid_params.curr_value = 0;
        // Angle Z need to be manually set before setting the command to restart
        wall_calibration_turn_command = RUN;
        wall_calibration_in_position = false;
        break;
    }
    }
}

void mpu6050_turn_left(){
    int angle = -90;
    mpu6050_turn_pid_params.target_value = angle + angleZ;
    motor_speed_task_command = RUN;
    robot_move_type = TURN;
    mpu6050_turn_command = RESTART;
    while (1){
        if (mpu6050_turn_in_position == true){
            mpu6050_turn_in_position = false;
            robot_move_type = DO_NOT_MOVE;
            pid_motors[0].target_value = 0;
            pid_motors[1].target_value = 0;
            ESP_LOGI(MAIN_LOG_TAG, "Turning in position");
            break;
        }
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mpu6050_turn_right(){
    int angle = 90;
    mpu6050_turn_pid_params.target_value = angle + angleZ;
    motor_speed_task_command = RUN;
    robot_move_type = TURN;
    mpu6050_turn_command = RESTART;
    while (1){
        if (mpu6050_turn_in_position == true){
            mpu6050_turn_in_position = false;
            robot_move_type = DO_NOT_MOVE;
            pid_motors[0].target_value = 0;
            pid_motors[1].target_value = 0;
            ESP_LOGI(MAIN_LOG_TAG, "Turning in position");
            break;
        }
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vl53_front_center_move_until(void *arg){
    int distance = 100;
    motor_speed_task_command = RUN;
    robot_move_type = STRAIGHT;
    mpu6050_move_straight_command = RESTART;
    while (1){
        // Arbitrary added a 20mm!!! Test again!!!
        if (vl53[0].distance < distance + 20){
            // ESP_LOGI(MAIN_LOG_TAG, "Distance: %u", vl53.distance[0]);
            mpu6050_move_straight_command = STOP;
            robot_move_type = DO_NOT_MOVE;
            pid_motors[0].target_value = 0;
            pid_motors[1].target_value = 0;
            break;
        }
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);

}

void vl53_wall_calibration(void *arg){
    robot_move_type = WALL_CALIBRATION;
    wall_calibration_turn_command = RESTART;
    motor_speed_task_command = RUN;
    while(1){
        if (wall_calibration_in_position == true){
            wall_calibration_in_position = false;
            robot_move_type = DO_NOT_MOVE;
            pid_motors[0].target_value = 0;
            pid_motors[1].target_value = 0;
            break;
        }   
    } 
        vTaskDelete(NULL);
}

void debug_task(void *arg){
    while(1){
        // ESP_LOGI(MAIN_LOG_TAG, "Global X %f", global_x);
        // ESP_LOGI(MAIN_LOG_TAG, "Distance %lld", delta_time);


        // Print the whole rgbc as a 3x4 matrix
        // printf("\033[H\033[J");
        // printf("Angle Z: %.2f\n", angleZ);
        // printf("vl53.distance: %u\t%u\t%u\t%u\t%u\n", vl53[0].distance, vl53[1].distance, vl53[2].distance, vl53[3].distance, vl53[4].distance);
        // printf("Sensor 1: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[0], r[0], g[0], b[0]);
        // printf("Sensor 2: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[1], r[1], g[1], b[1]);
        // printf("Sensor 3: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[2], r[2], g[2], b[2]);
        // fflush(stdout);


        // for (int i = 0; i < 2; i++)
        // {
        // ESP_LOGI("M0", "Output: %f, Error: %f, Target Speed: %f, Current Speed: %f, Delta Count: %d, P Term: %f, I Term: %f, D Term: %f", pid_motors[0].output, pid_motors[0].err, pid_motors[0].target_value, pid_motors[0].curr_value, encoder_pulses[0].delta_count, pid_motors[0].pTerm, pid_motors[0].iTerm, pid_motors[0].dTerm);
        // printf("Output:%f,Error:%f,TargetSpeed:%f,CurrentSpeed:%f,DeltaCount:%d,P_Term:%f,I_Term:%f,D_Term:%f\r\n", 
        //        pid_motors[0].output, pid_motors[0].err, pid_motors[0].target_value, pid_motors[0].curr_value, 
        //        encoder_pulses[0].delta_count, pid_motors[0].pTerm, pid_motors[0].iTerm, pid_motors[0].dTerm);

        // printf("Output:%f,Error:%f,TargetSpeed:%f,CurrentSpeed:%f,DeltaCount:%d,P_Term:%f,I_Term:%f,D_Term:%f\r\n", 
        //         pid_motors[1].output, pid_motors[1].err, pid_motors[1].target_value, pid_motors[1].curr_value, 
        //         encoder_pulses[1].delta_count, pid_motors[1].pTerm, pid_motors[1].iTerm, pid_motors[1].dTerm);

        // print out mpu6050 pid params
        // printf("MPU6050 Output:%f,Error:%f,TargetAngle:%f,CurrentAngle:%f,P_Term:%f,I_Term:%f,D_Term:%f\r\n", 
        //         mpu6050_pid_params.output, mpu6050_pid_params.err, mpu6050_pid_params.target_value, angleZ, 
        //         mpu6050_pid_params.pTerm, mpu6050_pid_params.iTerm, mpu6050_pid_params.dTerm);
        
        // // print the debug statement for wall calibratiuon
        // printf("Wall_Calibration_Output:%f,Error:%f,TargetAngle:%f,P_Term:%f,I_Term:%f,D_Term:%f\r\n", 
        //     wall_calibration_pid_params.output, wall_calibration_pid_params.err, wall_calibration_pid_params.target_value,
        //     wall_calibration_pid_params.pTerm, wall_calibration_pid_params.iTerm, wall_calibration_pid_params.dTerm);


        // print out v left and v right
        // ESP_LOGI("Velocity", "M0:%f M1:%f", v_left, v_right);

        // ESP_LOGI("M1", "Output: %f, Error: %f, Target Speed: %f, Current Speed: %f, Delta Count: %d, P Term: %f, I Term: %f, D Term: %f", pid_motors[1].output, pid_motors[1].err, pid_motors[1].target_value, pid_motors[1].curr_value, encoder_pulses[1].delta_count, pid_motors[1].pTerm, pid_motors[1].iTerm, pid_motors[1].dTerm);
        // }
        // ESP_LOGI(MAIN_LOG_TAG, "Encoder Count 1 %d Encoder Count 2 %d", encoder_pulses[0].curr_count, encoder_pulses[1].curr_count);

        // ESP_LOGI(MAIN_LOG_TAG, "Time taken: %lld us", delta_time);

        // ESP_LOGI("MPU6050", "Output: %f, Error: %f, Target Angle: %f, Current Angle: %f, P Term: %f, I Term: %f, D Term: %f", mpu6050_pid_params.output, mpu6050_pid_params.err, mpu6050_pid_params.target_value, angleZ, mpu6050_pid_params.pTerm, mpu6050_pid_params.iTerm, mpu6050_pid_params.dTerm);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void scratchpad_func(){
    // motor_speed_task_command = RUN;
    // robot_move_type = REVERSE;
    // mpu6050_move_straight_command = RESTART;
    while (1){
        // mcpwm_comparator_set_compare_value(servo, SERVO_ANGLE_TO_COMPARATOR(175));
        // vTaskDelay(pdMS_TO_TICKS(3000));
        // mcpwm_comparator_set_compare_value(servo, SERVO_ANGLE_TO_COMPARATOR(0));
        vTaskDelay(pdMS_TO_TICKS(3000));

        // ESP_LOGI("Main", "Moving until wall");
        // vl53_front_center_move_until(100);
        // vTaskDelay(pdMS_TO_TICKS(50));

        // ESP_LOGI("Main", "Performing wall calibration");
        // vl53_wall_calibration();
        // vTaskDelay(pdMS_TO_TICKS(50));

        // ESP_LOGI("Main", "Turning");
        // mpu6050_turn(-90);
        // vTaskDelay(pdMS_TO_TICKS(50));

        // vl53_front_center_move_until(100);
        // mpu6050_turn(-90);
        // ESP_LOGI(MAIN_LOG_TAG, "Distance_left:%u,Distancce_right:%u,Error:%f", vl53.distance[VL53_FRONT_LEFT], vl53.distance[VL53_FRONT_RIGHT], wall_calibration_pid_params.err);
            
        }
        return;
}

// Helper functions;
void move_straight_gyro_H(){
    motor_speed_task_command = RUN;
    robot_move_type = STRAIGHT;
    mpu6050_move_straight_command = RESTART;
}

void move_reverse_gyro_H(){
    motor_speed_task_command = RUN;
    robot_move_type = REVERSE;
    mpu6050_move_straight_command = RESTART;
}

void stop_robot_H(){
    // mpu6050_move_straight_command = STOP;
    robot_move_type = DO_NOT_MOVE;
    pid_motors[0].target_value = 0;
    pid_motors[1].target_value = 0;
}


// restructure needed
void robot_wall_calibration_H(){
    robot_move_type = WALL_CALIBRATION;
    wall_calibration_turn_command = RESTART;
    motor_speed_task_command = RUN;
    ulTaskNotifyTakeIndexed(WALL_CALIBRATION_SLOT, pdTRUE, portMAX_DELAY);
    stop_robot_H();
}

void move_reverse_gyro_x_amount(){
    motor_speed_task_command = RUN;
    robot_move_type = REVERSE;
    mpu6050_move_straight_command = RESTART;
    // while(1){
        // float target = global_x -
    // }
}

void servo_blink_extrude(void *args){
    vTaskSuspend(maze_logic_task_handler);
    vTaskSuspend(uart_control_task_handler);
    vTaskSuspend(color_sensor_compute_task_handler);
    taskCommand_t motor_speed_task_command_backup = motor_speed_task_command;
    taskCommand_t mpu6050_move_straight_command_backup = mpu6050_move_straight_command;
    taskCommand_t mpu6050_turn_command_backup = mpu6050_turn_command;
    taskCommand_t wall_calibration_turn_command_backup = wall_calibration_turn_command;
    robot_move_t robot_move_type_backup = robot_move_type;
    stop_robot_H();
    gpio_set_level(LED3, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(LED3, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(LED3, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(LED3, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(LED3, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
    gpio_set_level(LED3, 0);

    if( servo_pos == 0 ){
        mcpwm_comparator_set_compare_value(servo, SERVO_ANGLE_TO_COMPARATOR(175));
        servo_pos = 175;
    }else{
        mcpwm_comparator_set_compare_value(servo, SERVO_ANGLE_TO_COMPARATOR(0));
        servo_pos = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(4000));

    motor_speed_task_command = motor_speed_task_command_backup;
    mpu6050_move_straight_command = mpu6050_move_straight_command_backup;
    mpu6050_turn_command = mpu6050_turn_command_backup;
    wall_calibration_turn_command = wall_calibration_turn_command_backup;
    robot_move_type = robot_move_type_backup;
    vTaskResume(maze_logic_task_handler);
    vTaskResume(uart_control_task_handler);

    vTaskDelay(3000);
    vTaskResume(color_sensor_compute_task_handler);
    vTaskDelete(NULL);

}

void maze_logic(){
    switch (robot_state){
        case ROBOT_MAKE_DECISION:{
            if (vl53[VL53_FRONT_CENTER].detection == WALL && vl53[VL53_LEFT].detection == WALL &&  vl53[VL53z_RIGHT].detection == WALL){
                robot_state = ROBOT_MOVE_REVERSE_UNTIL_EMPTY;
            }else if (vl53[VL53_FRONT_CENTER].detection == WALL && vl53[VL53_LEFT].detection == EMPTY &&  vl53[VL53z_RIGHT].detection == WALL){
                robot_state = ROBOT_TURN_LEFT;
            }else if (vl53[VL53_FRONT_CENTER].detection == WALL && vl53[VL53_LEFT].detection == WALL &&  vl53[VL53z_RIGHT].detection == EMPTY){
                robot_state = ROBOT_TURN_RIGHT;

            }else if (vl53[VL53_FRONT_CENTER].detection == WALL && vl53[VL53_LEFT].detection == EMPTY &&  vl53[VL53z_RIGHT].detection == EMPTY){
                if ((esp_random() % 2) == 0){
                    robot_state = ROBOT_TURN_LEFT;
                }else{
                    robot_state = ROBOT_TURN_LEFT;

                }
            }else if (vl53[VL53_FRONT_CENTER].detection == EMPTY){
                robot_state = ROBOT_MOVE_STRAIGHT;
            }
            break;
        case ROBOT_TURN_LEFT :{
            
        }
        }default:{
            break;
        }
    }
}

esp_err_t vl53l0x_setup_address_init(i2c_master_bus_handle_t bus1){
    vl53l0x_handle = &vl53l0x_default; // Global variable in vl53l0x.c
    esp_err_t status = ESP_OK;
    /* Address Changing Start */
    
    vTaskDelay(pdMS_TO_TICKS(100));

    if (i2c_master_probe(bus1, VL53_ADDR_0, 100) ==  ESP_OK){
        ESP_LOGI(MAIN_LOG_TAG, "VL53 Address already changed to 0x28. No need to change");
    }else{
        setAddress(VL53_ADDR_0, 0);
        ESP_LOGI(MAIN_LOG_TAG, "VL53 address not changed to 0x28. Changing address");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    gpio_set_level(X_SHUT_1, 1); // Select sensor 1
    vTaskDelay(pdMS_TO_TICKS(10));
    status |= setAddress(VL53_ADDR_1, 1); // Set sensor 1 to address 1

    gpio_set_level(X_SHUT_2, 1); // Select sensor 2
    vTaskDelay(pdMS_TO_TICKS(10));
    status |= setAddress(VL53_ADDR_2, 2); // Set sensor 2 to address 2

    gpio_set_level(X_SHUT_3, 1); // Select sensor 3
    vTaskDelay(pdMS_TO_TICKS(10));
    status |= setAddress(VL53_ADDR_3, 3); // Set sensor 3 to address 3

    gpio_set_level(X_SHUT_4, 1); // Select sensor 4
    vTaskDelay(pdMS_TO_TICKS(10));
    status |= setAddress(VL53_ADDR_4, 4); // Set sensor 4 to address 4

    /* Address Changing End */

    /* Initialize all 5 sensors start */
    for (int i = 0; i < 5;i++){
        vl53l0x_handle = vl53l0x_arr[i];
        if (init()){
            startContinuous(0);
            ESP_LOGI(TOF_LOG_TAG, "VL53L0X %d Initialized", i);
        }else{
            ESP_LOGE(TOF_LOG_TAG, "Error Initializing VL53L0X %d, exiting", i);
            return ESP_FAIL;
        };
    }
    return status;
    /* Initialize all 5 sensors end */
}

void pcnt_init(){
    // Setup pulse counter unit
    pcnt_unit_config_t pcnt_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
        .flags.accum_count = true
    };
    pcnt_new_unit(&pcnt_config, &encoder1);
    pcnt_new_unit(&pcnt_config, &encoder2);

    pcnt_unit_handle_t encoder_handle_array[2] = {encoder1, encoder2};

    const gpio_num_t encoder_array[2][2] = {
        {ENCODER1_PIN_A, ENCODER1_PIN_B},
        {ENCODER2_PIN_A, ENCODER2_PIN_B}
    };

    // Configure 2 encoder channels
    for (int i = 0; i < 2; i++){
        // Setup pulse counter channel, each unit has 2 channels
        pcnt_chan_config_t channel_a_config = {
            .edge_gpio_num = encoder_array[i][0],
            .level_gpio_num = encoder_array[i][1]};
        pcnt_chan_config_t channel_b_config = {
            .edge_gpio_num = encoder_array[i][1],
            .level_gpio_num = encoder_array[i][0]};
        pcnt_channel_handle_t channel_a = NULL, channel_b = NULL;
        
        pcnt_new_channel(encoder_handle_array[i], &channel_a_config, &channel_a);
        pcnt_new_channel(encoder_handle_array[i], &channel_b_config, &channel_b);

        // Set edge and level actions for pulse counter channels
        pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    }

    /* Add watch points, PCNT_HIGH_LIMIT and PCNT_LOW_LIMIT which is required to
    accumulate count when the counter overflow or underflow
    See: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/pcnt.html#compensate-overflow-loss
    */
    pcnt_unit_add_watch_point(encoder1, PCNT_HIGH_LIMIT);
    pcnt_unit_add_watch_point(encoder1, PCNT_LOW_LIMIT);
    pcnt_unit_add_watch_point(encoder2, PCNT_HIGH_LIMIT);
    pcnt_unit_add_watch_point(encoder2, PCNT_LOW_LIMIT);

    pcnt_unit_enable(encoder1);
    pcnt_unit_enable(encoder2);
    pcnt_unit_clear_count(encoder1);
    pcnt_unit_clear_count(encoder2);
    pcnt_unit_start(encoder1);
    pcnt_unit_start(encoder2);
    return;
}

void mcpwm_init(){
    // Configure mcpwm timer
    mcpwm_timer_handle_t timer0 = NULL;
    mcpwm_timer_config_t timer0_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, // 160Mhz default clock source
        .resolution_hz = TIMER_RESOLUTION,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, //Count up down for symetric waveform to reduce harmonics when driving DC motors
        .period_ticks = COUNTER_PERIOD
    };
    mcpwm_new_timer(&timer0_config, &timer0);

    // Configure mcpwm operator
    mcpwm_oper_handle_t operator0 = NULL;
    mcpwm_operator_config_t operator0_config = {
        .group_id = 0,
    };
    mcpwm_new_operator(&operator0_config, &operator0);
    mcpwm_operator_connect_timer(operator0, timer0);

    // Configure mcpwm comparator
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tep = true
    };
    mcpwm_new_comparator(operator0, &comparator_config, &motor1);
    mcpwm_new_comparator(operator0, &comparator_config, &motor2);

    mcpwm_gen_handle_t generator1 = NULL, generator2 = NULL;
    mcpwm_generator_config_t generator1_config = {.gen_gpio_num = MOTOR_1_PWM};
    mcpwm_generator_config_t generator2_config = {.gen_gpio_num = MOTOR_2_PWM};

    mcpwm_new_generator(operator0, &generator1_config, &generator1);
    mcpwm_new_generator(operator0, &generator2_config, &generator2);

    /* 
    Configure the correct wave characteristics to interface with motor driver
    Dual Edge Symmetric Waveform - Active Low (Modified to active high)
    https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html#dual-edge-symmetric-waveform-active-low
    */ 
    mcpwm_generator_set_actions_on_compare_event(generator1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor1, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor1, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(generator2,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor2, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor2, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    // Set each motor to off initially
    mcpwm_comparator_set_compare_value(motor1, 0);
    mcpwm_comparator_set_compare_value(motor2, 0);

    mcpwm_timer_enable(timer0);
    mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP);
    return;

}

void servo_init(){
    // Configure mcpwm timer
    mcpwm_timer_handle_t servo_timer = NULL;
    mcpwm_timer_config_t servo_timer_config = {
        .group_id = 1,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, // 160Mhz default clock source
        .resolution_hz = SERVO_PWM_RESOLUTION,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP, //Count up down for symetric waveform to reduce harmonics when driving DC motors
        .period_ticks = SERVO_COUNTER_PERIOD
    };
    mcpwm_new_timer(&servo_timer_config, &servo_timer);

    // Configure mcpwm operator
    mcpwm_oper_handle_t operator1 = NULL;
    mcpwm_operator_config_t operator1_config = {
        .group_id = 1,
    };
    mcpwm_new_operator(&operator1_config, &operator1);
    mcpwm_operator_connect_timer(operator1, servo_timer);

    // Configure mcpwm comparator
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tep = true
    };
    mcpwm_new_comparator(operator1, &comparator_config, &servo);

    mcpwm_gen_handle_t generator1 = NULL;
    mcpwm_generator_config_t generator1_config = {.gen_gpio_num = SERVO_PWM_PIN};

    mcpwm_new_generator(operator1, &generator1_config, &generator1);

    /* 
    Configure the correct wave characteristics to interface with servo motor
    Single Edge Asymmetric Waveform - Active High
    https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html#single-edge-asymmetric-waveform-active-high
    */ 

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, servo, MCPWM_GEN_ACTION_LOW)));

    // ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1,
    //         MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_FULL, MCPWM_GEN_ACTION_LOW)));
    // ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1,
    //         MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, servo, MCPWM_GEN_ACTION_HIGH)));

    // Set each motor to off initially
    mcpwm_comparator_set_compare_value(servo, 0);

    mcpwm_timer_enable(servo_timer);
    mcpwm_timer_start_stop(servo_timer, MCPWM_TIMER_START_NO_STOP);
    return;
   
}

void app_main(){

    /* GPIO Configuration and Initial State */
    gpio_config_t xshut_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_BIT_MASK,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&xshut_config);
    // Turn all sansor off
    gpio_set_level(X_SHUT_1, 0);
    gpio_set_level(X_SHUT_2, 0);
    gpio_set_level(X_SHUT_3, 0);
    gpio_set_level(X_SHUT_4, 0);

    gpio_set_level(COLOR_SENSOR_LED, 1);
    
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);

    int calibrate_sensors = gpio_get_level(BUTTON_PIN);
    if (calibrate_sensors == 0){
        ESP_LOGI(TCS34725_LOG_TAG, "Sensor calibration requessted!");
    }

    nvs_flash_init();

    mcpwm_init();
    
    pcnt_init();

    servo_init();
    mcpwm_comparator_set_compare_value(servo, SERVO_ANGLE_TO_COMPARATOR(0));

    init_uart();

    initialize_pid_controller(&pid_motors[0], 0, MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD, MOTOR_SPEED_MAX_INTEGRAL, MOTOR_SPEED_MIN_INTEGRAL);
    initialize_pid_controller(&pid_motors[1], 0, MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD, MOTOR_SPEED_MAX_INTEGRAL, MOTOR_SPEED_MIN_INTEGRAL);
    initialize_pid_controller(&mpu6050_pid_params, 0, PID_STRAIGHT_KP, PID_STRAIGHT_KI, PID_STRAIGHT_KD, PID_STRAIGHT_MAX_INTEGRAL, PID_STRAIGHT_MIN_INTEGRAL);
    initialize_pid_controller(&mpu6050_turn_pid_params, 0, PID_TURN_KP, PID_TURN_KI, PID_TURN_KD, PID_TURN_MAX_INTEGRAL, PID_TURN_MIN_INTEGRAL);
    initialize_pid_controller(&wall_calibration_pid_params, 0, WALL_CALIBRATION_KP, WALL_CALIBRATION_KI, WALL_CALIBRATION_KD, WALL_CALIBRATION_MAX_INTEGRAL, WALL_CALIBRATION_MIN_INTEGRAL);

    /* I2C BUS 0 Start */
    /* BUS 0 - TCS34725 with TCA95 I2C MUX */
    i2c_master_bus_config_t i2c_mst_config_bus_0 = {
        .sda_io_num = I2C_SDA_PIN_BUS_0,
        .scl_io_num = I2C_SCL_PIN_BUS_0,
        .i2c_port = 0,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
        .trans_queue_depth = 0 // synchronous mode
    };
    i2c_device_config_t device_config_bus_0 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCS34725_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    }; 
    i2c_master_bus_handle_t i2c_mst_handle_bus_0;

    i2c_new_master_bus(&i2c_mst_config_bus_0, &i2c_mst_handle_bus_0);
    
    i2c_master_bus_add_device(i2c_mst_handle_bus_0, &device_config_bus_0, &tcs34725_handle);
    device_config_bus_0.device_address = TCA95_ADDR; // Add TCA9548A I2C Multiplexer
    i2c_master_bus_add_device(i2c_mst_handle_bus_0, &device_config_bus_0, &tca95_handle);
   
    /* I2C BUS 0 End */

    /* I2C BUS 1 Start */
    /* BUS 1 - 5 x VLX53L0X, MPU6050 */
    i2c_master_bus_config_t i2c_mst_config_bus_1 = {
        .sda_io_num = I2C_SDA_PIN_BUS_1,
        .scl_io_num = I2C_SCL_PIN_BUS_1,
        .i2c_port = 1,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .trans_queue_depth = 0 // synchronous mode
    };
    i2c_device_config_t device_config_bus_1 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    }; 
    i2c_master_bus_handle_t i2c_mst_handle_bus_1;

    i2c_new_master_bus(&i2c_mst_config_bus_1, &i2c_mst_handle_bus_1);

    i2c_master_bus_add_device(i2c_mst_handle_bus_1, &device_config_bus_1, &vl53l0x_default);

    device_config_bus_1.device_address = VL53_ADDR_0;
    i2c_master_bus_add_device(i2c_mst_handle_bus_1, &device_config_bus_1, &vl53l0x_sensor0);
    device_config_bus_1.device_address = VL53_ADDR_1;
    i2c_master_bus_add_device(i2c_mst_handle_bus_1, &device_config_bus_1, &vl53l0x_sensor1);
    device_config_bus_1.device_address = VL53_ADDR_2;
    i2c_master_bus_add_device(i2c_mst_handle_bus_1, &device_config_bus_1, &vl53l0x_sensor2);
    device_config_bus_1.device_address = VL53_ADDR_3;
    i2c_master_bus_add_device(i2c_mst_handle_bus_1, &device_config_bus_1, &vl53l0x_sensor3);
    device_config_bus_1.device_address = VL53_ADDR_4;
    i2c_master_bus_add_device(i2c_mst_handle_bus_1, &device_config_bus_1, &vl53l0x_sensor4);
    device_config_bus_1.device_address = MPU6050_ADDR;
    i2c_master_bus_add_device(i2c_mst_handle_bus_1, &device_config_bus_1, &mpu6050_handle);

    /* I2C BUS 1 End */
    
    /* Sensor Initialization Start */
    if (vl53l0x_setup_address_init(i2c_mst_handle_bus_1) != ESP_OK){
        ESP_LOGE(TOF_LOG_TAG, "VL53L0X Initialization Failed");
        return;
    }

    if (i2c_master_probe(i2c_mst_handle_bus_0, TCA95_ADDR, 100) ==  ESP_OK){
        ESP_LOGI(MAIN_LOG_TAG, "TCA9548A Found");
    }else{
        ESP_LOGE(MAIN_LOG_TAG, "TCA9548A Not Found");
    }

    if (tcs34725_init_multi(&tcs34725_handle, &tca95_handle) == ESP_OK){
        ESP_LOGI(TCS34725_LOG_TAG, "TCS34725 Initialized");
    }else{
        ESP_LOGE(TCS34725_LOG_TAG, "TCS34725 Initialization Failed");
        return;
    }
    if (mpu6050_init(&mpu6050_handle) != ESP_OK){
        ESP_LOGE("MPU6050", "MPU6050 Initialization Failed");
        return;
    }
    /* Sensor Initialization End */

    /* Initialize ESP32 Hardware TImer Start */
    gptimer_config_t timer_config ={
        .clk_src = GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // One tick = 1us
        .intr_priority = 0
    };
    gptimer_handle_t sensor_sampling_timer;
    gptimer_new_timer(&timer_config, &sensor_sampling_timer);

    gptimer_alarm_config_t sensor_alarm_config = {
        .reload_count = 0, // counter will reload with 0 on alarm event
        .alarm_count = 4000, // period = 4ms
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    gptimer_set_alarm_action(sensor_sampling_timer, &sensor_alarm_config);

    gptimer_event_callbacks_t  callback_config = {
        .on_alarm = sensors_sampling_isr_handler
    };
    gptimer_register_event_callbacks(sensor_sampling_timer, &callback_config, NULL);

    /* Initialize ESP32 Hardware TImer End */
    
    // Sensor detection tasks
    // xTaskCreatePinnedToCore(wall_detection_task, "wall_detection_task", 2048, NULL, 3, &wall_detection_task_handler, 1);
    xTaskCreatePinnedToCore(color_sensor_compute_task, "color_sensor_sampling_task", 2048, NULL, 1, &color_sensor_compute_task_handler, 1);

    // Sensor sampling task
    xTaskCreatePinnedToCore(sensors_sampling_task, "sensor_sampling_task", 2048, NULL, 10, &sensor_sampling_handler, 0);
    gptimer_enable(sensor_sampling_timer);
    gptimer_start(sensor_sampling_timer);

    ESP_LOGI(MAIN_LOG_TAG, "Waiting for button press or uart character to start..."); 
    // wait until button is pressed or uart character is received
    uint8_t temp;
    while (gpio_get_level(BUTTON_PIN) == 1){
        int len = uart_read_bytes(UART_NUM_2, &temp, 1, pdMS_TO_TICKS(50));
        if (len > 0){
            ESP_LOGI(MAIN_LOG_TAG, "UART character received: %c", temp);
            break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    xTaskCreatePinnedToCore(uart_control_task, "uart_control_task", 2048, (void*)&uart_task_args, 10, &uart_control_task_handler, 1);

    xTaskCreatePinnedToCore(debug_task, "debug_task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(scratchpad_func, "scratchpadfunc", 2048, NULL, 1, NULL, 1);
    
    if (calibrate_sensors == 0){
        xTaskCreatePinnedToCore(color_calibration_task, "color_calibration_task", 2048, NULL, 5, &calibrate_color_sensor_task_handler, 1);
            // Write to NVS
    }else{
        ESP_LOGI(TCS34725_LOG_TAG, "Loading calibration data from nvs...");
        nvs_read(normalized_calibration, sizeof(normalized_calibration));
        ESP_LOGI(TCS34725_LOG_TAG, "Successful");
        ESP_LOGI(TCS34725_LOG_TAG, "Read %f from NVS success!", normalized_calibration[0][0]);
        nvs_flash_deinit();
        print_normalized(normalized_calibration);
        xTaskNotifyGive(uart_control_task_handler);

        xTaskCreatePinnedToCore(maze_logic, "maze_logic", 2048, NULL, 10, &maze_logic_task_handler, 1);
    }

    

    // Core 0 - time crtitical sensor sampling task
    // Core 1 - Main program logic, debug task, etc.

}