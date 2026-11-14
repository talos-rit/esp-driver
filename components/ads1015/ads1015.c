#include "ads1015.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "signal_bus.h"

#include <stdio.h>

#define TAG "ADS1015"

// A conversion at the slowest rate (128 SPS) takes about 8 ms. No RDY for this
// long means the RDY wire or the chip has a problem
#define ADS1015_RDY_TIMEOUT_MS 100

static void IRAM_ATTR ads1015_isr(void* arg) {
  ads1015_handle_t* handle = (ads1015_handle_t*)arg;
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (handle->task) {
    xTaskNotifyFromISR(handle->task, 0, eNoAction, &higher_priority_task_woken);
  }

  if (higher_priority_task_woken) {
    portYIELD_FROM_ISR();
  }
}

static uint16_t config_for_pair(uint16_t config_reg, int pair) {
  config_reg &= ~(0b111 << ADS1015_MUX_SHIFT);
  config_reg |= (pair == 0 ? ADS1015_MUX_AIN0_AIN1 : ADS1015_MUX_AIN2_AIN3) << ADS1015_MUX_SHIFT;
  return config_reg | (1 << ADS1015_OS_BIT);  // writing it starts a conversion
}

// Reads the watched pairs in turn, one single-shot conversion each. Every
// reading belongs to the pair that was selected when it was started.
static void adc_task(void* arg) {
  ads1015_handle_t* handle = (ads1015_handle_t*)arg;
  bool watched[2] = {handle->pair_motor[0] != 0, handle->pair_motor[1] != 0};
  int pair = watched[0] ? 0 : 1;
  bool rdy_missing = false;
  // Readings in a row past a threshold on each pair, so one noisy sample does
  // not stop the motors
  int over_count[2] = {0, 0};

  ads1015_write_register(handle, ADS1015_CONFIG, config_for_pair(handle->config_reg, pair));
  while (1) {
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ADS1015_RDY_TIMEOUT_MS)) == 0) {
      // No RDY: this pair is not being watched. Say so once and keep retrying
      if (!rdy_missing) {
        ESP_LOGE(TAG, "0x%02X: no RDY for %d ms, overcurrent on motor %d/%d NOT watched. Check its RDY wire.",
                 handle->i2c_addr, ADS1015_RDY_TIMEOUT_MS, handle->pair_motor[0], handle->pair_motor[1]);
        rdy_missing = true;
      }
      over_count[0] = over_count[1] = 0;
      ads1015_write_register(handle, ADS1015_CONFIG, config_for_pair(handle->config_reg, pair));
      continue;
    }
    if (rdy_missing) {
      ESP_LOGI(TAG, "0x%02X: RDY back, watching again", handle->i2c_addr);
      rdy_missing = false;
    }

    uint16_t raw;
    if (ads1015_read_register(handle, ADS1015_CONVERSION, &raw) == ESP_OK) {
      int16_t value = ((int16_t)raw) >> 4;
      bool over = value >= CONFIG_ADS1015_HIGH_THRESH || value <= CONFIG_ADS1015_LOW_THRESH;
      if (!over) {
        over_count[pair] = 0;
      } else if (over_count[pair] < CONFIG_ADS1015_TRIP_SAMPLES) {
        over_count[pair]++;
        // Log once per trip, readings come in thousands per second
        if (over_count[pair] == CONFIG_ADS1015_TRIP_SAMPLES) {
          ESP_LOGW(TAG, "Overcurrent on motor %d (ADC 0x%02X %s, reading %d)", handle->pair_motor[pair],
                   handle->i2c_addr, pair == 0 ? "A0-A1" : "A2-A3", value);
        }
      }

      // Keeps signalling while the pair stays past the threshold
      if (over_count[pair] == CONFIG_ADS1015_TRIP_SAMPLES) {
        xEventGroupSetBits(g_motor_events, CURRENT_BIT(handle->pair_motor[pair] - 1));
      }
    }

    // Next watched pair, then start its conversion
    if (watched[0] && watched[1]) {
      pair = !pair;
    }
    ads1015_write_register(handle, ADS1015_CONFIG, config_for_pair(handle->config_reg, pair));
  }
}

esp_err_t ads1015_init(ads1015_handle_t* handle, const ads1015_config_t* config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  for (int p = 0; p < 2; p++) {
    if (config->pair_motor[p] > CURRENT_NUM_BITS) {
      ESP_LOGE(TAG, "0x%02X pair %d: motor %d, must be 0 to %d", config->i2c_addr, p, config->pair_motor[p],
               CURRENT_NUM_BITS);
      return ESP_ERR_INVALID_ARG;
    }
  }

  handle->task = NULL;
  handle->i2c_addr = config->i2c_addr;
  handle->pair_motor[0] = config->pair_motor[0];
  handle->pair_motor[1] = config->pair_motor[1];

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->i2c_addr,
      .scl_speed_hz = config->i2c_speed_hz,
  };

  esp_err_t ret = i2c_master_bus_add_device(config->bus_handle, &dev_config,
                                            &handle->dev_handle);
  if (ret != ESP_OK) {
    return ret;
  }

  // Set comparator threshold registers for RDY mode
  ret = ads1015_write_register(handle, ADS1015_HIGH_THRESH_REG, 0x8000);
  if (ret != ESP_OK) {
    return ret;
  }
  ret = ads1015_write_register(handle, ADS1015_LOW_THRESH_REG, 0x0000);
  if (ret != ESP_OK) {
    return ret;
  }

  // Write config register
  uint16_t config_reg = ads1015_build_config(
      ADS1015_MUX_AIN0_AIN1, ADS1015_PGA_4_096V, ADS1015_MODE_SINGLESHOT,
      config->adc_data_rate, ADS1015_COMP_TRADITIONAL, ADS1015_COMP_ACTIVE_LOW,
      ADS1015_COMP_NON_LATCHING, ADS1015_COMP_ASSERT_1,
      false  // don't conversions immediately
  );

  ret = ads1015_write_register(handle, ADS1015_CONFIG, config_reg);
  if (ret != ESP_OK) {
    return ret;
  }

  handle->config_reg = config_reg;

  if (config->pair_motor[0] == 0 && config->pair_motor[1] == 0) {
    ESP_LOGI(TAG, "0x%02X initialized, no pairs watched", config->i2c_addr);
    return ESP_OK;
  }

  // Configure alert GPIO and interrupt service
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_NEGEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = 1ULL << config->rdy_gpio,
      .pull_up_en = GPIO_PULLUP_ENABLE,  // RDY is open-drain, it only pulls low
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    return ret;
  }
  ret = gpio_isr_handler_add(config->rdy_gpio, ads1015_isr, handle);
  if (ret != ESP_OK) {
    return ret;
  }

  // One task per chip, woken by its own RDY
  char name[16];
  snprintf(name, sizeof(name), "ads1015_%02x", config->i2c_addr);
  if (xTaskCreate(adc_task, name, 4096, handle, 10, &handle->task) != pdPASS) {
    gpio_isr_handler_remove(config->rdy_gpio);
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "0x%02X watching A0-A1 = motor %d, A2-A3 = motor %d (0 = none), RDY GPIO%d", config->i2c_addr,
           config->pair_motor[0], config->pair_motor[1], config->rdy_gpio);

  return ESP_OK;
}

esp_err_t ads1015_read_register(ads1015_handle_t* handle,
                                ads1015_register_t reg, uint16_t* data) {
  if (handle == NULL || data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t rx_buf[2];

  esp_err_t ret =
      i2c_master_transmit_receive(handle->dev_handle, (uint8_t*)&reg, 1, rx_buf,
                                  2, ADS1015_I2C_MASTER_TIMEOUT_MS);

  if (ret == ESP_OK) {
    *data = (rx_buf[0] << 8) | rx_buf[1];
  }

  return ret;
}

esp_err_t ads1015_write_register(ads1015_handle_t* handle,
                                 ads1015_register_t reg, uint16_t data) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t write_buf[3];
  write_buf[0] = reg;
  write_buf[1] = (data >> 8) & 0xFF;  // MSB first
  write_buf[2] = data & 0xFF;         // LSB
  return i2c_master_transmit(handle->dev_handle, write_buf, sizeof(write_buf),
                             ADS1015_I2C_MASTER_TIMEOUT_MS);
}
