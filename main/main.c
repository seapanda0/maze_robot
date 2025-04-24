#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <stdlib.h> // For abs()

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

/* Task command status */
taskCommand_t motor_speed_task_command = STOP; 
taskCommand_t mpu6050_move_straight_command = STOP; 
taskCommand_t mpu6050_turn_command = STOP; 
mpu6050_move_t mpu6050_move_type = STRAIGHT;

/* Variables measuring timing */
int64_t prev_time = 0;
int64_t curr_time = 0;
int64_t delta_time = 0;

/*Timer Variables*/
uint32_t cycle_count = 0;

/*Sensor Readings*/
float angleZ = 0;
uint32_t r[3] = {0}, g[3] = {0}, b[3] = {0}, c[3] = {0};
uint16_t distances[5] = {0};
int encoder_1_count = 0, encoder_2_count = 0;

/*PID Structs*/
pid_controller_t pid_motors[2] = {0};
pid_controller_t mpu6050_pid_params = {0};
encoder_pulses_t encoder_pulses[2] = {0};
pid_controller_t mpu6050_turn_pid_params = {0};
bool mpu6050_turn_in_position = false;

/*Motor controls*/
float basespeed_m0 = 1.25;
float basespeed_m1 = 1.25;

/*Manual Overrides Via UART*/
char keyboard_input = 0;
uart_control_input_t uart_task_args = {
    .pid_motors_m1 = &pid_motors[0],
    .pid_motors_m2 = &pid_motors[1],
    .motor_speed_task_command = &motor_speed_task_command,
    .motor1_comparator = &motor1,
    .motor2_comparator = &motor2,
};

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

        // Every 2 cycle , 8ms, sample wheel encoder and perform PID control
        if (cycle_count % 2 == 0)
        {
            switch (mpu6050_move_type)
            {
            case STRAIGHT:{
                mpu6050_move_straight_pid();
                pid_motors[0].target_value = basespeed_m0 + mpu6050_pid_params.output;
                pid_motors[1].target_value = basespeed_m0 - mpu6050_pid_params.output;
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
            }
            }
            pcnt_unit_get_count(encoder1, &encoder_pulses[0].curr_count);
            pcnt_unit_get_count(encoder2, &encoder_pulses[1].curr_count);
            motor_pid_speed_control();
        }

        // Every 6th cycle, 24ms, sample tcs34725
        if (cycle_count % 6 == 0){
            tcs34725_read_raw_multi(&tcs34725_handle, &tca95_handle, c, r, g, b);  // Implement error handling!!!
        }
        // Every 9th cycle, 36ms, sample vl53l0x
        if (cycle_count % 9 == 0){
            for (int i = 0; i < 5; i++){
                status |= i2c_master_transmit_receive(*vl53l0x_arr[i], &write_buf, 1, read_buf, 2, I2C_TIMEOUT_MS);
                status |= i2c_master_transmit(*vl53l0x_arr[i], write_buf_clear, 2 , I2C_TIMEOUT_MS);
                if (status == ESP_OK) {        
                    distances[i] = read_buf[1] | (read_buf[0] << 8);
                } // Implement error handling!!!
            }
            xTaskNotifyGive(wall_detection_task_handler);
            
        }
        // prev_time = curr_time;
        // curr_time = esp_timer_get_time();
        // delta_time = curr_time - prev_time;
        // if (delta_time > 3000){
        //     ESP_LOGI(MAIN_LOG_TAG, "Time taken: %lld us", delta_time);
        // }
    }
}

void wall_detection_task(void *arg){
    while(1){
        // Wait for the task to be notified
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Perform wall detection logic here
        // For example, check the distance from the sensors and take action
        for (int i = 0; i < 5; i++){
            if (distances[i] < 1000){ // Example threshold
                ESP_LOGI(MAIN_LOG_TAG, "Wall detected by sensor %d", i);
            }
        }
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
            if (in_position_cycle_count > 5){
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

void mpu6050_turn(int angle){
    mpu6050_turn_pid_params.target_value = angle + angleZ;
    mpu6050_turn_command = RESTART;
    mpu6050_move_type = TURN;
    motor_speed_task_command = RUN;
    while (1){
        if (mpu6050_turn_in_position == true){
            mpu6050_move_type = DO_NOT_MOVE;
            pid_motors[0].target_value = 0;
            pid_motors[1].target_value = 0;
            break;
        }
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vl53_front_center_move_until(int distance){
    motor_speed_task_command = RUN;
    mpu6050_move_straight_command = RESTART;
    mpu6050_move_type = STRAIGHT;
    while (1){
        // Arbitrary added a 20mm!!! Test again!!!
        if (distances[0] < distance + 20){
            ESP_LOGI(MAIN_LOG_TAG, "Distance: %u", distances[0]);
            mpu6050_move_straight_command = STOP;
            mpu6050_move_type = DO_NOT_MOVE;
            pid_motors[0].target_value = 0;
            pid_motors[1].target_value = 0;
            break;
        }
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void debug_task(void *arg){
    while(1){
        // Print the whole rgbc as a 3x4 matrix
        // printf("\033[H\033[J");
        // printf("Angle Z: %.2f\n", angleZ);
        // printf("Distances: %u\t%u\t%u\t%u\t%u\n", distances[0], distances[1], distances[2], distances[3], distances[4]);
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
        

        // ESP_LOGI("M1", "Output: %f, Error: %f, Target Speed: %f, Current Speed: %f, Delta Count: %d, P Term: %f, I Term: %f, D Term: %f", pid_motors[1].output, pid_motors[1].err, pid_motors[1].target_value, pid_motors[1].curr_value, encoder_pulses[1].delta_count, pid_motors[1].pTerm, pid_motors[1].iTerm, pid_motors[1].dTerm);
        // }
        // ESP_LOGI(MAIN_LOG_TAG, "Encoder Count 1 %d Encoder Count 2 %d", encoder_pulses[0].curr_count, encoder_pulses[1].curr_count);

        // ESP_LOGI(MAIN_LOG_TAG, "Time taken: %lld us", delta_time);

        // ESP_LOGI("MPU6050", "Output: %f, Error: %f, Target Angle: %f, Current Angle: %f, P Term: %f, I Term: %f, D Term: %f", mpu6050_pid_params.output, mpu6050_pid_params.err, mpu6050_pid_params.target_value, angleZ, mpu6050_pid_params.pTerm, mpu6050_pid_params.iTerm, mpu6050_pid_params.dTerm);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void scratchpad_func(){
    vl53_front_center_move_until(100);
    mpu6050_turn(-90);
    while (1){
            ESP_LOGI(MAIN_LOG_TAG, "Distance: %u", distances[0]);
            
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        }
        return;
}

void maze_logic(){
    while (1){

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

    initialize_pid_controller(&pid_motors[0], 0, MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD, MOTOR_SPEED_MAX_INTEGRAL, MOTOR_SPEED_MIN_INTEGRAL);
    initialize_pid_controller(&pid_motors[1], 0, MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD, MOTOR_SPEED_MAX_INTEGRAL, MOTOR_SPEED_MIN_INTEGRAL);
    initialize_pid_controller(&mpu6050_pid_params, 0, PID_STRAIGHT_KP, PID_STRAIGHT_KI, PID_STRAIGHT_KD, PID_STRAIGHT_MAX_INTEGRAL, PID_STRAIGHT_MIN_INTEGRAL);
    initialize_pid_controller(&mpu6050_turn_pid_params, 0, PID_TURN_KP, PID_TURN_KI, PID_TURN_KD, PID_TURN_MAX_INTEGRAL, PID_TURN_MIN_INTEGRAL);

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

    mcpwm_init();
    
    pcnt_init();

    servo_init();

    init_uart();

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
    
    xTaskCreatePinnedToCore(sensors_sampling_task, "sensor_sampling_task", 2048, NULL, 10, &sensor_sampling_handler, 0);
    gptimer_enable(sensor_sampling_timer);
    gptimer_start(sensor_sampling_timer);


    xTaskCreatePinnedToCore(uart_control_task, "uart_control_task", 2048, (void*)&uart_task_args, 1, NULL, 1);

    
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    ESP_LOGI(MAIN_LOG_TAG, "Waiting for button press to start..."); 
    // wait until button is pressed
    while (gpio_get_level(BUTTON_PIN) == 1){
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(1500));

    xTaskCreatePinnedToCore(debug_task, "debug_task", 2048, NULL, 1, NULL, 1);
    // xTaskCreatePinnedToCore(scratchpad_func, "scratchpadfunc", 2048, NULL, 1, NULL, 1);

    // xTaskCreatePinnedToCore(wall_detection_task, "wall_detection_task", 2048, NULL, 1, &, &wall_detection_task_handler, 1);
    // xTaskCreatePinnedToCore(maze_logic, "maze_logic", 2048, NULL, 10, NULL, 1);


    // Core 0 - time crtitical sensor sampling task
    // Core 1 - Main program logic, debug task, etc.

}