#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tcs34725.h"
#include "vlx53l0x.h"
#include "i2c_configuration.h"
#include "mpu6050.h"

/* I2C Device Handlers */
i2c_master_dev_handle_t tcs34725_handle = NULL, tca95_handle = NULL;
i2c_master_dev_handle_t mpu6050_handle = NULL;
i2c_master_dev_handle_t vl53l0x_sensor0, vl53l0x_sensor1, vl53l0x_sensor2, vl53l0x_sensor3, vl53l0x_sensor4;
i2c_master_dev_handle_t *vl53l0x_arr[5] = {&vl53l0x_sensor0, &vl53l0x_sensor1, &vl53l0x_sensor2, &vl53l0x_sensor3, &vl53l0x_sensor4};

/* FreeRTOS Task Handlers */
TaskHandle_t sensor_sampling_handler = NULL;

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
            for (int i = 0; i < 5; i++){
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

void debug_task(void *arg){
    while(1){
        printf("\033[H\033[J");
        // Print the whole rgbc as a 3x4 matrix
        printf("Angle Z: %.2f\n", angleZ);
        printf("Distances: %u\t%u\t%u\t%u\t%u\n", distances[0], distances[1], distances[2], distances[3], distances[4]);
        printf("Sensor 1: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[0], r[0], g[0], b[0]);
        printf("Sensor 2: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[1], r[1], g[1], b[1]);
        printf("Sensor 3: C: %lu\tR: %lu\tG: %lu\tB: %lu\n", c[2], r[2], g[2], b[2]);
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t vl53l0x_setup_address_init(){
    vl53l0x_handle = &vl53l0x_sensor0; // Global variable in vl53l0x.c
    esp_err_t status = ESP_OK;
    /* Address Changing Start */
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    gpio_set_level(X_SHUT_1, 1); // Select sensor 1
    vTaskDelay(pdMS_TO_TICKS(10));
    status |= setAddress(VL53_ADDR_1, 1); // Set sensor 1 to address 1

    gpio_set_level(X_SHUT_2, 1); // Select sensor 2
    vTaskDelay(pdMS_TO_TICKS(10));
    status |= setAddress(VL53_ADDR_2, 2); // Set sensor 2 to address 2

    gpio_set_level(X_SHUT_3, 1); // Select sensor 3
    vTaskDelay(pdMS_TO_TICKS(10));
    status |= setAddress(VL53_ADDR_3, 3); // Set sensor 3 to address 3

    gpio_set_level(X_SHUT_4, 1); // Select sensor 3
    vTaskDelay(pdMS_TO_TICKS(10));
    status |= setAddress(VL53_ADDR_4, 4); // Set sensor 3 to address 3

    gpio_set_level(X_SHUT_0, 1); // Power on sensor 0, no need change address
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Address Changing End */

    /* Initialize all 5 sensors start */
    for (int i = 0; i < 5; i++){
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

void app_main(){

    /* GPIO Configuration and Initial State */
    gpio_config_t xshut_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << X_SHUT_0) | (1ULL << X_SHUT_1) | (1ULL << X_SHUT_2) | (1ULL << X_SHUT_3) | (1ULL << X_SHUT_4),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    gpio_config(&xshut_config);
    // Turn all sansor off
    gpio_set_level(X_SHUT_0, 0);
    gpio_set_level(X_SHUT_1, 0);
    gpio_set_level(X_SHUT_2, 0);
    gpio_set_level(X_SHUT_3, 0);
    gpio_set_level(X_SHUT_4, 0);

    /* I2C BUS 0 Start */
    /* BUS 0 - TCS34725 with TCA95 I2C MUX */
    i2c_master_bus_config_t i2c_mst_config_bus_0 = {
        .sda_io_num = I2C_SDA_PIN_BUS_0,
        .scl_io_num = I2C_SCL_PIN_BUS_0,
        .i2c_port = 0,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
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
        .device_address = VL53_ADDR_0,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    }; 
    i2c_master_bus_handle_t i2c_mst_handle_bus_1;

    i2c_new_master_bus(&i2c_mst_config_bus_1, &i2c_mst_handle_bus_1);
    
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
    if (vl53l0x_setup_address_init() != ESP_OK){
        ESP_LOGE(TOF_LOG_TAG, "VL53L0X Initialization Failed");
        return;
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

    xTaskCreatePinnedToCore(debug_task, "debug_task", 2048, NULL, 5, NULL, 1);

    // uint32_t r[3] = {0}, g[3] = {0}, b[3] = {0}, c[3] = {0};
    
    // i2c_master_transmit(tca95_handle, &TCA95_CH1, 1, I2C_TIMEOUT_MS); // Enable TCA9548A channel 1
    while(1){
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
        vTaskDelay(pdMS_TO_TICKS(30));
        // ESP_LOGI(TCS34725_LOG_TAG, "C: %lu, R: %lu, G: %lu, B: %lu", c, r, g, b);
        // ESP_LOGI(TOF_LOG_TAG, "Time taken: %lld us", curr_time - prev_time);

    }
}