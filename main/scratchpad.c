



//     for (int i = 500; i <= 3700; i=i+20){
            //         mcpwm_comparator_set_compare_value(servo, i);
            //         ESP_LOGI(MAIN_LOG_TAG, "Servo %d", i);]
            //     }

            // ESP_LOGI(MAIN_LOG_TAG, "Servo 10 degrees");
            // mcpwm_comparator_set_compare_value(servo, SERVO_ANGLE_TO_COMPARATOR(0));
            // vTaskDelay(pdMS_TO_TICKS(1000)); 
            // ESP_LOGI(MAIN_LOG_TAG, "Servo 40 degrees");
            // mcpwm_comparator_set_compare_value(servo, SERVO_ANGLE_TO_COMPARATOR(170));
            // vTaskDelay(pdMS_TO_TICKS(1000)); 

            // gpio_set_level(GPIO_NUM_5, 0);
            // vTaskDelay(pdMS_TO_TICKS(2500));
            // gpio_set_level(GPIO_NUM_5, 1);
            // vTaskDelay(pdMS_TO_TICKS(2500));



            /* Encoder troubleshooting*/

            // int count1, count2;
            // int prev_count1 = 0, prev_count2 = 0;
            // prev_time = esp_timer_get_time();
            // pcnt_unit_get_count(encoder1, &prev_count1);
            // pcnt_unit_get_count(encoder2, &prev_count2);
            // set_motor_power(800, 800);
            // esp_rom_delay_us(4000);
            // pcnt_unit_get_count(encoder1, &count1);
            // pcnt_unit_get_count(encoder2, &count2);
            // curr_time = esp_timer_get_time();
            // int diff1 = count1 - prev_count1;
            // int diff2 = count2 - prev_count2;
            // ESP_LOGI(MAIN_LOG_TAG, "Encoder Count 1 %d Encoder Count 2 %d Time Taken %lld", diff1, diff2, curr_time - prev_time);
            // ESP_LOGI(MAIN_LOG_TAG, "Encoder Count 1 %d Encoder Count 2 %d", count1, count2);
        
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