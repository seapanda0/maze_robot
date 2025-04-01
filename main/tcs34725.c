#include "tcs34725.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

// Constants
const uint8_t tca95_channel[3] = {TCA95_CH1, TCA95_CH2, TCA95_CH3};
const uint8_t tcs34725_command_cdata = TCS34725_COMMAND | TCS34725_CDATA;

/* Public Functions */
esp_err_t tcs34725_init(i2c_master_dev_handle_t *tcs34725_handle){
  
  // Check if the device is connected
  uint8_t buf8 = TCS34725_COMMAND | TCS34725_ID;
  esp_err_t status = i2c_master_transmit_receive(*tcs34725_handle, &buf8, 1, &buf8, 1, I2C_TIMEOUT_MS);
  if (status != ESP_OK) {
    ESP_LOGE(TCS34725_LOG_TAG, "TCS34725 Not Detected!, check connections");
    return status;
  }else{
    ESP_LOGI(TCS34725_LOG_TAG, "TCS34725 Detected with ID: 0x%02X", buf8);
  }

  // Set the integration time
  uint8_t buf8_arr[2] = {TCS34725_COMMAND | TCS34725_ATIME, TCS34725_INTEGRATIONTIME};
  status |= i2c_master_transmit(*tcs34725_handle, buf8_arr, 2, I2C_TIMEOUT_MS);

  // Set gain
  buf8_arr[0]= TCS34725_COMMAND | TCS34725_CONTROL;
  buf8_arr[1]= TCS34725_AGAIN;
  status |= i2c_master_transmit(*tcs34725_handle, buf8_arr, 2, I2C_TIMEOUT_MS);
  status |= i2c_master_transmit(*tcs34725_handle, buf8_arr, 2, I2C_TIMEOUT_MS);
  
  // Set PON bit in enable register
  buf8_arr[0]= TCS34725_COMMAND | TCS34725_ENABLE;
  buf8_arr[1]= TCS34725_ENABLE_PON;
  status |= i2c_master_transmit(*tcs34725_handle, buf8_arr, 2, I2C_TIMEOUT_MS);
  
  esp_rom_delay_us(2400); // With PON is enables, 2.4ms is required as per datasheet

  // Set AEN bit in enable register
  buf8_arr[1]= TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
  status |= i2c_master_transmit(*tcs34725_handle, buf8_arr, 2, I2C_TIMEOUT_MS);

  esp_rom_delay_us(2400); // With PON is enables, 2.4ms is required as per datasheet

  return status;
}

esp_err_t tcs34725_init_multi(i2c_master_dev_handle_t *tcs34725_handle, i2c_master_dev_handle_t *tca95_handle){
  esp_err_t status = ESP_OK;
  status |= i2c_master_transmit(*tca95_handle, tca95_channel, 1, I2C_TIMEOUT_MS); // Enable TCA9548A channel 1
  status |= tcs34725_init(tcs34725_handle);
  status |= i2c_master_transmit(*tca95_handle, tca95_channel + 1, 1, I2C_TIMEOUT_MS); // Enable TCA9548A channel 2
  status |= tcs34725_init(tcs34725_handle);
  status |= i2c_master_transmit(*tca95_handle, tca95_channel + 2, 1, I2C_TIMEOUT_MS); // Enable TCA9548A channel 2
  status |= tcs34725_init(tcs34725_handle);
  return status;
}


/* Read a single sensor connected to the microcontroller */
esp_err_t tcs34725_read_raw (i2c_master_dev_handle_t *tcs34725_handle, uint8_t * buf){
  uint8_t temp8 = TCS34725_COMMAND | TCS34725_CDATA;
  esp_err_t status = i2c_master_transmit_receive(*tcs34725_handle, &temp8, 1,  buf, 8, I2C_TIMEOUT_MS);
  return status;
}

void raw_to_rgbc(uint8_t * buf, uint32_t *c, uint32_t *r, uint32_t *g, uint32_t *b){
  *c = (buf[1] << 8) | buf[0];
  *r = (buf[3] << 8) | buf[2];
  *g = (buf[5] << 8) | buf[4];
  *b = (buf[7] << 8) | buf[6];
}

/* Read 3 TCS34725 connected to TCA95 I2C Mux*/
esp_err_t tcs34725_read_raw_multi (
  i2c_master_dev_handle_t *tcs34725_handle, 
  i2c_master_dev_handle_t *tca95_handle, 
  uint32_t *c,
  uint32_t *r,
  uint32_t *g,
  uint32_t *b) {

  uint8_t buf[8] = {0};
  esp_err_t status = ESP_OK;

  // int64_t prev_time = esp_timer_get_time();

  status |= i2c_master_transmit(*tca95_handle, tca95_channel, 1, I2C_TIMEOUT_MS); // Enable TCA9548A channel 1
  status |= i2c_master_transmit_receive(*tcs34725_handle, &tcs34725_command_cdata, 1,  buf, 8, I2C_TIMEOUT_MS);
  raw_to_rgbc(buf, c, r, g, b);
  
  status |= i2c_master_transmit(*tca95_handle, tca95_channel + 1, 1, I2C_TIMEOUT_MS); // Enable TCA9548A channel 1
  status |= i2c_master_transmit_receive(*tcs34725_handle, &tcs34725_command_cdata, 1,  buf, 8, I2C_TIMEOUT_MS);
  raw_to_rgbc(buf, c+1, r+1, g+1, b+1);

  status |= i2c_master_transmit(*tca95_handle, tca95_channel + 2, 1, I2C_TIMEOUT_MS); // Enable TCA9548A channel 1
  status |= i2c_master_transmit_receive(*tcs34725_handle, &tcs34725_command_cdata, 1,  buf, 8, I2C_TIMEOUT_MS);
  raw_to_rgbc(buf, c+2, r+2, g+2, b+2);

  // int64_t curr_time = esp_timer_get_time();

  // ESP_LOGI(TCS34725_LOG_TAG, "Time taken to read all 3 sensors: %lld us", curr_time - prev_time);
  return status;
}