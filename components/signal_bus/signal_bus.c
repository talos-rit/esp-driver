#include "signal_bus.h"
#include "esp_log.h"

#define TAG "signal_bus"

EventGroupHandle_t g_motor_events = NULL;

esp_err_t signal_bus_init(void) {
    ESP_LOGI(TAG, "Initializing signal bus...");

    g_motor_events = xEventGroupCreate();

    if (g_motor_events == NULL) {
        return ESP_FAIL;
    } else{
        return ESP_OK;
    }
}
