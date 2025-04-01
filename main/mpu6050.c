#include "mpu6050.h"

const char *TAG = "MPU6050";

int64_t dt = 0, currtime = 0, prevtime = 0;
float calibrationZ = 0; 
float temp = 0;


// List of registers and command to configure mpu6050
uint8_t i2c_data[] = {
    MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_VAL,
    MPU6050_GYRO_CONFIG, MPU6050_GYRO_CONFIG_VAL,
    MPU6050_SMPRT_DIV, MPU6050_SMPRT_DIV_VAL,
    MPU6050_CONFIG, MPU6050_CONFIG_VAL
};

int16_t get_GyroZ_raw(i2c_master_dev_handle_t *mpu6050_handle){
    uint8_t data = MPU6050_GYRO_ZOUT_H;
    uint8_t gyroZ_raw[2] = {0};
    
    // int64_t start_time = esp_timer_get_time();
    
    // Need to implement error handling if this function fails
    i2c_master_transmit_receive(*
        mpu6050_handle, &data, sizeof(data), gyroZ_raw, sizeof(gyroZ_raw), -1);
    // int64_t end_time = esp_timer_get_time();

    // ESP_LOGI(TAG, "%lld", end_time - start_time);

    int16_t gyroZ_signed = ((gyroZ_raw[0] << 8) | gyroZ_raw[1]);
    // float gyroZ = (float)gyroZ_signed / GYRO_LSB_SENSITIVITY;
    // ESP_LOGI(TAG, "%f", gyroZ);
    return gyroZ_signed;
}

float get_temperature(i2c_master_dev_handle_t mpu6050_handle){
    uint8_t data = MPU6050_TEMP_OUT_H;
    uint8_t temperature_raw[2] = {0};
    i2c_master_transmit_receive(mpu6050_handle, &data, sizeof(data), temperature_raw, sizeof(temperature_raw), -1);
    int16_t temperature_signed = ((temperature_raw[0]<<8) | temperature_raw[1]);
    float temperature_float = (float)temperature_signed / 340.0 + 36.53;
    // ESP_LOGI(TAG, "%f", temperature_float); 
    return temperature_float;
}

void print_debug_mpu6050(void *arg){
    float* angleZ = (float*)arg;
    while(1){
        ESP_LOGI(TAG, "%f",  *angleZ    );
    }
}

void mpu6050_updateZ(i2c_master_dev_handle_t *mpu6050_handle, float* angleZ){
    currtime = esp_timer_get_time();
    int16_t gyroZ = get_GyroZ_raw(mpu6050_handle);
    dt = currtime - prevtime;
    prevtime = currtime;
    temp = ((float)gyroZ - calibrationZ)/GYRO_LSB_SENSITIVITY* (float)dt;
    // if (temp < 0.07 && temp >- 0.07){
    //     temp = 0;
    // }
    *angleZ = *angleZ + temp;

    // int64_t execution_time = esp_timer_get_time();
    // ESP_LOGI(TAG, "Log time: %lld", execution_time - currtime);
    
    // ESP_LOGI(TAG, "%f", calibrationZ);
}

esp_err_t mpu6050_init(i2c_master_dev_handle_t *mpu6050_handle){
    esp_err_t status = ESP_OK;
    // Send everything in i2c_data[]
    for (int i = 0; i < sizeof(i2c_data); i += 2)
    {
        uint8_t data[2] = {i2c_data[i], i2c_data[i + 1]};
        status |= i2c_master_transmit(*mpu6050_handle, data, sizeof(data), 500);
        if ( status != ESP_OK ){
            ESP_LOGE(TAG, "Failed to initialize MPU6050, Check Connections");
            return status;
        }
    }
    ESP_LOGI(TAG, "MPU6050 Initialized");
    
    ESP_LOGI(TAG, "Calibrating MPU6050, do not move the sensor");
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int i = 0; i<MPU6050_CALIBRATION_SAMPLE_SIZE; i++){
        int16_t gyroZ = get_GyroZ_raw(mpu6050_handle); 
        calibrationZ += (float)gyroZ;
    }
    calibrationZ = (calibrationZ/MPU6050_CALIBRATION_SAMPLE_SIZE);
    ESP_LOGI(TAG, "%f", calibrationZ);
    ESP_LOGI(TAG, "MPU6050 Calibration Done");
    prevtime = esp_timer_get_time();
    return status;
};