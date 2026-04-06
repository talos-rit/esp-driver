#include "limit_switch.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "signal_bus.h"

#define TAG "limit_switch"

static void IRAM_ATTR limit_switch_isr(void* arg) {
  limit_switch_config_t* config = (limit_switch_config_t*)arg;
  BaseType_t higher_priority_task_woken = pdFALSE;

  int level = gpio_get_level(config->limit_gpio);
  if (level) {
    // Limit switch released, clear corresponding bit
    xEventGroupClearBitsFromISR(g_motor_events, LIMIT_0);
  } else {
    // Limit switch triggered, set corresponding bit
    xEventGroupSetBitsFromISR(g_motor_events, LIMIT_0,
                              &higher_priority_task_woken);
  }
      
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

esp_err_t limit_switch_init(const limit_switch_config_t* config) {
  if (!config) {
    return ESP_ERR_INVALID_ARG;
  }

  // Configure GPIO interrupt service
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_ANYEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = 1ULL << config->limit_gpio,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };

  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_isr_handler_add(config->limit_gpio, limit_switch_isr, (void*)config));

  return ESP_OK;
}