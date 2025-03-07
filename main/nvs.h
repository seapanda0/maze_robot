#ifndef NVS_H
#define NVS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

void nvs_write(float *data_ptr, size_t data_size);
void nvs_read(float *data_ptr, size_t data_size);

#define STORAGE_NAMESPACE "storage"

#endif