#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "uart_control.h"

#define UART_BAUD_RATE 921600
#define BUF_SIZE 1024
#define UART_TX UART_PIN_NO_CHANGE
#define UART_RX GPIO_NUM_3

const uart_port_t uart_num = UART_NUM_2;

void uart_control_task(void *arg)
{
    uart_control_input_t *uart_control_input = (uart_control_input_t *)arg;
    char data;
    // size_t len = 0;
    while (1){
        
        int len = uart_read_bytes(uart_num, &data, 1, pdMS_TO_TICKS(50));
        if (len > 0){
            gpio_set_level(LED3, 1);
            switch (data)
            {
            case 'w':{
                *(uart_control_input->motor_speed_task_command) = RUN;
                *(uart_control_input->robot_move_type) = MANUAL;
                uart_control_input->pid_motors_m1->target_value = 2.5;
                uart_control_input->pid_motors_m2->target_value = 2.5;
                break;
            }
            case 's':{
                *(uart_control_input->motor_speed_task_command) = RUN;
                *(uart_control_input->robot_move_type) = MANUAL;
                uart_control_input->pid_motors_m1->target_value = -2.5;
                uart_control_input->pid_motors_m2->target_value = -2.5;
                break;
            }case 'a':{
                *(uart_control_input->motor_speed_task_command) = RUN;
                *(uart_control_input->robot_move_type) = MANUAL;
                uart_control_input->pid_motors_m1->target_value = -2.5;
                uart_control_input->pid_motors_m2->target_value = 2.5;
                break;
            }case 'd':{
                *(uart_control_input->motor_speed_task_command) = RUN;
                *(uart_control_input->robot_move_type) = MANUAL;
                uart_control_input->pid_motors_m1->target_value = 2.5;
                uart_control_input->pid_motors_m2->target_value = -2.5;
                break;
            }case ' ':{
                // *(uart_control_input->motor_speed_task_command) = STOP;
                *(uart_control_input->robot_move_type) = MANUAL;
                uart_control_input->pid_motors_m1->target_value = 0;
                uart_control_input->pid_motors_m2->target_value = 0;
                break;}
            case 'k':{
                esp_restart();
                break;
            }
            }
        }else{
            gpio_set_level(LED3, 0);
        }
    }
}

void init_uart()
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    uart_driver_install(uart_num, BUF_SIZE*2, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    

}

