#ifndef TCS34725
#define TCS34725

#include "driver/i2c_master.h"
#include "i2c_configuration.h"

/*I2C Configuration Define Start*/


#define TCS34725_LOG_TAG "TCS34725"

/*I2C Configuration Define End*/

/* TCA95 I2C Mux Constants Start */
#define TCA95_ADDR 0x70 // TCA9548A I2C Multiplexer Address
#define TCA95_CH1 0x02 // TCA9548A Channel 1
#define TCA95_CH2 0x04 // TCA9548A Channel 2
#define TCA95_CH3 0x08 // TCA9548A Channel 3
/* TCA95 I2C Mux Constants End */


/* Public Functions */
esp_err_t tcs34725_init(i2c_master_dev_handle_t *tcs34725_handle);

esp_err_t tcs34725_init_multi(i2c_master_dev_handle_t *tcs34725_handle, i2c_master_dev_handle_t *tca95_handle);

/* Read a single sensor connected to the microcontroller */
esp_err_t tcs34725_read_raw (i2c_master_dev_handle_t *tcs34725_handle, uint8_t * buf);

/* Read 3 TCS34725 connected to TCA95 I2C Mux
c, r, g, b are of length 3 array
c[0], r[0], g[0], b[0] are the values of TCS34725 1
*/
esp_err_t tcs34725_read_raw_multi (
    i2c_master_dev_handle_t *tcs34725_handle, 
    i2c_master_dev_handle_t *tca95_handle, 
    uint32_t *c,
    uint32_t *r,
    uint32_t *g,
    uint32_t *b);
  


/*TCS34725 REGISTERS*/

#define TCS34725_ADDR 0x29
#define TCS34725_COMMAND 0x80 // Repeated byte protocol transaction

#define TCS34725_ENABLE 0x00
#define TCS34725_ATIME 0x01
#define TCS34725_ID 0x12
#define TCS34725_CONTROL 0x0F
#define TCS34725_CDATA 0x14

#define TCS34725_ENABLE_PON 0x01
#define TCS34725_ENABLE_AEN 0x02

#define TCS34725_INTEGRATIONTIME (0xF6) /**< 24.0ms - 10 cycles - Max Count: 10240 */
#define TCS34725_INTEGRATIONTIME_240MS (0x9C) /**< 240.0ms - 100 cycles - Max Count: 65535 */
#define TCS34725_AGAIN (0x00) // 1x gain
#endif