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

#define MAIN_LOG_TAG "MAIN"

/* GPIO Configuration */
#define GPIO_BIT_MASK (1ULL << X_SHUT_1) | (1ULL << X_SHUT_2) | (1ULL << X_SHUT_3) | (1ULL << X_SHUT_4) \
    | (1ULL << MOTOR_1_DIR) | (1ULL << MOTOR_2_DIR)

/* I2C Device Handlers */
i2c_master_dev_handle_t tcs34725_handle = NULL, tca95_handle = NULL;
i2c_master_dev_handle_t mpu6050_handle = NULL;
i2c_master_dev_handle_t vl53l0x_default, vl53l0x_sensor0, vl53l0x_sensor1, vl53l0x_sensor2, vl53l0x_sensor3, vl53l0x_sensor4;
i2c_master_dev_handle_t *vl53l0x_arr[5] = {&vl53l0x_sensor0, &vl53l0x_sensor1, &vl53l0x_sensor2, &vl53l0x_sensor3, &vl53l0x_sensor4};

/* PCNT Handlers */
pcnt_unit_handle_t encoder1 = NULL, encoder2 = NULL;

/* MCPWM Comparator Handlers */
mcpwm_cmpr_handle_t motor1 = NULL, motor2 = NULL;

/* FreeRTOS Task Handlers */
TaskHandle_t sensor_sampling_handler = NULL, move_straight_task_handler = NULL;

/* Task command status */
taskCommand_t move_straight_task_command = STOP; 

/* Variables measuring timing */
int64_t prev_time = 0;
int64_t curr_time = 0;

/*Timer Variables*/
uint32_t cycle_count = 0;

/*Sensor Readings*/
float angleZ = 0;
uint32_t r[3] = {0}, g[3] = {0}, b[3] = {0}, c[3] = {0};
uint16_t distances[5] = {0};

bool sensors_sampling_isr_handler (gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx){
    BaseType_t xTaskawaken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_sampling_handler, &xTaskawaken);

    vTaskNotifyGiveFromISR(move_straight_task_handler, &xTaskawaken);
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

        // Every cycle, 4ms, sample mpu6050
        mpu6050_updateZ(&mpu6050_handle , &angleZ);

        // Every 6th cycle, 24ms, sample tcs34725
        if (cycle_count % 6 == 0){
            tcs34725_read_raw_multi(&tcs34725_handle, &tca95_handle, c, r, g, b);  // Implement error handling!!!
        }
        // Every 9th cycle, 36ms, sample vl53l0x
        if (cycle_count % 9 == 0){
            for (int i = 0; i < 4; i++){
                status |= i2c_master_transmit_receive(*vl53l0x_arr[i], &write_buf, 1, read_buf, 2, I2C_TIMEOUT_MS);
                status |= i2c_master_transmit(*vl53l0x_arr[i], write_buf_clear, 2 , I2C_TIMEOUT_MS);
                if (status == ESP_OK) {        
                    distances[i] = read_buf[1] | (read_buf[0] << 8);
                } // Implement error handling!!!
            }
        }
        // prev_time = curr_time;
        // curr_time = esp_timer_get_time();
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

void move_straight_task(void *arg){
    pid_controller_t pid = {
        .target_value = 0,
        .kp = PID_STRAIGHT_KP,
        .ki = PID_STRAIGHT_KI,
        .kd = PID_STRAIGHT_KD,
        .integral_limit_max = PID_STRAIGHT_MAX_INTEGRAL,
        .integral_limit_min = PID_STRAIGHT_MIN_INTEGRAL
    };


    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        switch (move_straight_task_command){
            case RUN: {
            
                // mpu6050_pid_params->err = target_angle_Z - angleZ;
            
                // // Calculate P term
                // mpu6050_pid_params->pTerm = mpu6050_pid_params->kp * mpu6050_pid_params->err;
            
                // // Calculate I term
                // mpu6050_pid_params->integral += mpu6050_pid_params->err;
                // mpu6050_pid_params->iTerm = mpu6050_pid_params->ki * mpu6050_pid_params->integral;
                // if (mpu6050_pid_params->iTerm > mpu6050_pid_params->integral_limit_max)
                // {
                //     mpu6050_pid_params->iTerm = mpu6050_pid_params->integral_limit_max;
                // }
                // else if (mpu6050_pid_params->iTerm < mpu6050_pid_params->integral_limit_min)
                // {
                //     mpu6050_pid_params->iTerm = mpu6050_pid_params->integral_limit_min;
                // }
            
                // // Derivative not implemented
                
                // // Calculate output and if it is within tolerance, set output to 0
                // if (fabs(mpu6050_pid_params->err) < mpu6050_pid_params->tolerance){
                //     mpu6050_pid_params->output = 0;
                // } else {
                //     mpu6050_pid_params->output = mpu6050_pid_params->pTerm + mpu6050_pid_params->iTerm;
                // }

                break;
            }
            case STOP: {
                // set_motor_speed(0, 0);
                break;
            }case RESET: {
                pid.target_value = angleZ;
                
                break;
            }
        }
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

        // ESP_LOGI(MAIN_LOG_TAG, "Time taken: %lld us", curr_time - prev_time);


        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void scratchpad_func(){

        int count1, count2;
        int prev_count1 = 0, prev_count2 = 0;

        
        while (1){
            prev_time = esp_timer_get_time();
            pcnt_unit_get_count(encoder1, &prev_count1);
            pcnt_unit_get_count(encoder2, &prev_count2);
            set_motor_power(500, 500);
            esp_rom_delay_us(4000);
            set_motor_power(0, 0);
            pcnt_unit_get_count(encoder1, &count1);
            pcnt_unit_get_count(encoder2, &count2);
            curr_time = esp_timer_get_time();


            // Calculate the difference and speed
            int diff1 = count1 - prev_count1;
            int diff2 = count2 - prev_count2;
        
            // ESP_LOGI(MAIN_LOG_TAG, "Encoder Count 1 %d Encoder Count 2 %d", count1, count2);
            ESP_LOGI(MAIN_LOG_TAG, "Encoder Count 1 %d Encoder Count 2 %d Time Taken %lld", diff1, diff2, curr_time - prev_time);
        }
        
        
        // vTaskDelay(portMAX_DELAY); 

        // uint32_t r[3] = {0}, g[3] = {0}, b[3] = {0}, c[3] = {0};
    
        // i2c_master_transmit(tca95_handle, &TCA95_CH1, 1, I2C_TIMEOUT_MS); // Enable TCA9548A channel 1

        // gpio_set_level(MOTOR_1_DIR, 1);
        // gpio_set_level(MOTOR_2_DIR, 1);
        // mcpwm_comparator_set_compare_value(motor1, 400);
        // mcpwm_comparator_set_compare_value(motor2, 400);
        // ESP_LOGI(MAIN_LOG_TAG, "Motor 1 and 2 Forward");
        // vTaskDelay(pdMS_TO_TICKS(1500));


        // gpio_set_level(MOTOR_1_DIR, 0);
        // gpio_set_level(MOTOR_2_DIR, 0);
        // mcpwm_comparator_set_compare_value(motor1, 400);
        // mcpwm_comparator_set_compare_value(motor2, 400);
        // ESP_LOGI(MAIN_LOG_TAG, "Motor 1 and 2 Backward");
        // vTaskDelay(pdMS_TO_TICKS(1500));

        // prev_time = esp_timer_get_time();
        // tcs34725_read_raw_multi(&tcs34725_handle, &tca95_handle, c, r, g, b);
        // prev_time = curr_time;
        // curr_time = esp_timer_get_time();
        // Clear the previous output
        // printf("\033[H\033[J");
        // Print the whole rgbc as a 3x4 matrix
        // printf("Sensor 1: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[0], r[0], g[0], b[0]);
        // printf("Sensor 2: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[1], r[1], g[1], b[1]);
        // printf("Sensor 3: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[2], r[2], g[2], b[2]);
        // fflush(stdout);

        // tcs34725_read_raw(&tcs34725_handle, data_raw);
        // c = data_raw[0] | (data_raw[1] << 8);
        // r = data_raw[2] | (data_raw[3] << 8);
        // g = data_raw[4] | (data_raw[5] << 8);
        // b = data_raw[6] | (data_raw[7] << 8);
        // vTaskDelay(pdMS_TO_TICKS(3000));
        // ESP_LOGI(TCS34725_LOG_TAG, "C: %lu, R: %lu, G: %lu, B: %lu", c, r, g, b);
        // ESP_LOGI(TOF_LOG_TAG, "Time taken: %lld us", curr_time - prev_time);
        return;
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

    // gpio_set_level(X_SHUT_4, 1); // Select sensor 4
    // vTaskDelay(pdMS_TO_TICKS(10));
    // status |= setAddress(VL53_ADDR_4, 4); // Set sensor 3 to address 3

    /* Address Changing End */

    /* Initialize all 5 sensors start */
    for (int i = 0; i < 4; i++){
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

    mcpwm_init();
    
    pcnt_init();

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
    xTaskCreatePinnedToCore(move_straight_task, "sensor_sampling_task", 2048, NULL, 4, &move_straight_task_handler, 1);
    gptimer_enable(sensor_sampling_timer);
    gptimer_start(sensor_sampling_timer);

    xTaskCreatePinnedToCore(debug_task, "debug_task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(scratchpad_func, "debug_task", 2048, NULL, 1, NULL, 1);

    // Core 0 - time crtitical sensor sampling task
    // Core 1 - Main program logic, debug task, etc.

    // while(1){


    // }
}