#include "nvs.h"

void nvs_write(float *data_ptr, size_t data_size){
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_blob(my_handle, "n_cal", (void*)data_ptr, data_size));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    nvs_close(my_handle);
}

void nvs_read(float *data_ptr, size_t data_size){
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_get_blob(my_handle, "n_cal", (char*)data_ptr, &data_size));
    nvs_close(my_handle);
}