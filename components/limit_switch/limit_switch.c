#include "limit_switch.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "LIMIT_SWITCH"

static TaskHandle_t limit_switch_task_handle = NULL; 

static void IRAM_ATTR limit_switch_isr(void *arg){
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (limit_switch_task_handle){
        xTaskNotifyFromISR(
            limit_switch_task_handle,
            0,
            eNoAction,
            &higher_priority_task_woken
        );
    }

    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

void limit_switch_task(void *arg){
    while (1) {
        // Wait until GPIO interrupt fires
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        ESP_LOGW(TAG,"Limit switch hit!")
        // TODO: React to limit switch
    }
}

esp_err_t limit_switch_init(limit_switch_config_t *config) {
    ESP_LOGI(TAG, "Initializing limit switches...");

    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure GPIO interrupt service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << config->alert_gpio,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
  
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(config->rdy_gpio, limit_switch_isr, NULL));
    
    xTaskCreate(limit_switch_task, "limit_switch_task", 4096, NULL, 10, &limit_switch_task_handle);

    ESP_LOGI(TAG, "limit_switch initialized");

    return ESP_OK;
}