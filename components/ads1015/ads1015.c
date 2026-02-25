#include "ads1015.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "ADS1015"

static TaskHandle_t adc_task_handle = NULL;

static void IRAM_ATTR ads1015_isr(void *arg){
    BaseType_t higher_priority_task_woken = pdFALSE;

    xTaskNotifyFromISR(
        adc_task_handle,
        0,
        eNoAction,
        &higher_priority_task_woken
    );

    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

void adc_task(){
    while (1) {
        // Wait until interrupt fires
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGW(TAG, "Overcurrent E-stop triggered!!!"); // TODO: Implement E-stop
    }
}

void test_task(void *arg){
    ads1015_handle_t *handle = (ads1015_handle_t *)arg;
    while (1) {
        uint16_t raw;
        if (ads1015_read_register(handle, ADS1015_CONVERSION, &raw) == ESP_OK){
            int16_t value = ((int16_t)raw) >> 4;
            ESP_LOGI(TAG, "ADC Value: %d", value);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

esp_err_t ads_init(ads1015_handle_t *handle, const ads1015_config_t *config) {
    ESP_LOGI(TAG, "Initializing ADS1015...");

    if (!handle || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->alert_gpio = config->alert_gpio;

    i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->i2c_addr,
      .scl_speed_hz = config->i2c_speed_hz,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(config->bus_handle, &dev_config, &handle->dev_handle));

    // Set comparator threshold registers
    int16_t high_raw = CONFIG_ADS1015_COMPARATOR_HIGH_THRESH;
    int16_t low_raw  = CONFIG_ADS1015_COMPARATOR_LOW_THRESH;
    uint16_t high_thresh = (uint16_t)((uint16_t)high_raw << 4);
    uint16_t low_thresh  = (uint16_t)((uint16_t)low_raw  << 4);\
    ESP_ERROR_CHECK(ads1015_write_register(handle, ADS1015_HIGH_THRESH, high_thresh));
    ESP_ERROR_CHECK(ads1015_write_register(handle, ADS1015_LOW_THRESH, low_thresh));

    // Write config register
    uint16_t config_reg = ads1015_build_config(
        ADS1015_MUX_AIN0_AIN3,
        ADS1015_PGA_4_096V,
        ADS1015_MODE_CONTINUOUS,
        ADS1015_DR_3300SPS,
        ADS1015_COMP_WINDOW,
        ADS1015_COMP_ACTIVE_LOW,
        ADS1015_COMP_LATCHING,
        ADS1015_COMP_ASSERT_1,
        true // start conversions
    );

    ESP_ERROR_CHECK(ads1015_write_register(
      handle, 
      ADS1015_CONFIG, 
      config_reg
    ));

    // Start task that waits for interrupt
    xTaskCreate(adc_task, "adc_task", 4096, handle, 10, &adc_task_handle);
    xTaskCreate(test_task, "test_task", 4096, handle, 5, NULL);

    // Configure alert GPIO and interrupt service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << config->alert_gpio,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
  
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(config->alert_gpio, ads1015_isr, NULL));

    ESP_LOGI(TAG, "ADS1015 initialized");

    return ESP_OK;
}

esp_err_t ads1015_read_register(ads1015_handle_t *handle, ads1015_register_t reg, uint16_t *data) {
  if (handle == NULL || data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t rx_buf[2];

    esp_err_t ret = i2c_master_transmit_receive(
        handle->dev_handle,
        (uint8_t *)&reg,
        1,
        rx_buf,
        2,
        ADS1015_I2C_MASTER_TIMEOUT_MS
    );

    if (ret == ESP_OK) {
        *data = (rx_buf[0] << 8) | rx_buf[1];
    }

    return ret;
}

esp_err_t ads1015_write_register(ads1015_handle_t *handle, ads1015_register_t reg, uint16_t data) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t write_buf[3];
  write_buf[0] = reg;
  write_buf[1] = (data >> 8) & 0xFF;  // MSB first
  write_buf[2] = data & 0xFF;         // LSB
  return i2c_master_transmit(
    handle->dev_handle,
    write_buf,
    sizeof(write_buf),
    ADS1015_I2C_MASTER_TIMEOUT_MS);
}
