## Maze Robot

### Robot Specification
- ESP32 Microcontroller
- 5 x VL53L0X Time-of-Flight (ToF) ranging sensor (3 in front, left and right)
- 3 x TCS34725 Color Sensor, with TCA9548A I2C Mux
- MPU6050 IMU
- 2 x MG310 DC Motor with differential drive configuration
- MDD10A Cytron Motor Driver

### ESP32 Peripheral Configuration
- I2C Bus 0 - 3 x TCS34725, ICA9548 I2C Mux
- I2C Bus 1 - 5 x VL53L0X, MPU6050
- General Purpose Timer (GPTimer) with 4ms ISR for sensor sampling task
- MCPWM (Not yet implemented)
- PCNT (Not yet implemented)

### File Structure and Program Organization
- `main.c` is the main code for robot logic and peripheral initialization
- Sensor drivers' header and C file like `mpu6050.h` and `vlx53lox.c` contains sensor functions like initialization and reading
- Some time critical sensor sampling code are implemented in `main.c` instead of the sensor driver

### Building and flashing
The project is writted using [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) APIs. Activation of the ESP-IDF environment is required before using the `idf.py` command. See [Step 4. Set up the Environment Variables](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html#step-4-set-up-the-environment-variables)

To build and flash
```
git clone https://github.com/seapanda0/maze_robot.git
cd maze_robot
idf.py build
idf.py flash

```