#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define GYRO_LSB_SENSITIVITY 131000000.0 // 131 (from datasheet) * 1000000 (microsend per second)
#define MPU6050_CALIBRATION_SAMPLE_SIZE 2000

#define MPU6050_ADDR 0x68

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_1_VAL 0x00

#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_CONFIG_VAL 0x00

#define MPU6050_SMPRT_DIV 0X19
#define MPU6050_SMPRT_DIV_VAL 0x00

#define MPU6050_CONFIG 0x1A
#define MPU6050_CONFIG_VAL 0x00

#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_GYRO_ZOUT_H 0x47

// Public Functions
esp_err_t mpu6050_init(i2c_master_dev_handle_t *mpu6050_handle);
void mpu6050_updateZ(i2c_master_dev_handle_t *mpu6050_handle, float* angleZ);

#endif
