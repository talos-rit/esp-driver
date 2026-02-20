#include "ads1015.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "ADS1015"

static void IRAM_ATTR ads1015_isr(void *arg) {
    ads1015_handle_t *handle = (ads1015_handle_t *)arg;
    gpio_intr_disable(handle->alert_gpio); // Disable interrupt until this conversion is done

    if (handle->rdy_sem == NULL){
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t high_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(handle->rdy_sem, &high_task_woken);
    if (high_task_woken) {
        portYIELD_FROM_ISR();
    }
}

void adc_task(void *arg){
    ads1015_handle_t *handle = (ads1015_handle_t *)arg;
    gpio_intr_enable(handle->alert_gpio); // Enable interrupt now that the task will handle it

    while (1){
        if (xSemaphoreTake(handle->rdy_sem, portMAX_DELAY)){
            uint16_t raw;
            ads1015_read_register(handle, ADS1015_CONVERSION, &raw);

            raw >>= 4; // ADS1015 is 12-bit, so shift to get the actual value

            // ESP_LOGI(TAG, "ADC value: %u", raw);

            gpio_intr_enable(handle->alert_gpio); // Re-enable interrupt for next conversion
            ads1015_start_conversion(handle);
        }
    }
}

esp_err_t ads_init(ads1015_handle_t *handle, const ads1015_config_t *config) {
    ESP_LOGI(TAG, "Initializing ADS1015...");

    if (!handle || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->alert_gpio = config->alert_gpio; // Store alert GPIO in handle for ISR use

    i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->i2c_addr,
      .scl_speed_hz = config->i2c_speed_hz,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(config->bus_handle, &dev_config, &handle->dev_handle));

    // Set comparator threshold registers for RDY mode
    ESP_ERROR_CHECK(ads1015_write_register(handle, ADS1015_HIGH_THRESH, 0x8000));
    ESP_ERROR_CHECK(ads1015_write_register(handle, ADS1015_LOW_THRESH, 0x0000));

    // Write config register
    uint16_t config_reg = 0;

    config_reg |= (0b100 << 12);  // Have converter read from AIN0
    config_reg |= (0b001 << 9);   // Set gain to ±4.096V
    config_reg |= (1 << 8);       // Single shot mode
    config_reg |= (0b001 << 5);   // 250 samples per second
    config_reg |= (0 << 4);       // Traditional comparator
    config_reg |= (0 << 3);       // Active low
    config_reg |= (0 << 2);       // Non-latching
    config_reg |= (0b00);         // Assert after 1 conversion

    handle->config_reg = config_reg;

    ESP_ERROR_CHECK(ads1015_write_register(
      handle, 
      ADS1015_CONFIG, 
      config_reg
    ));

    // Create binary semaphore
    handle->rdy_sem = xSemaphoreCreateBinary();
    assert(handle->rdy_sem != NULL);

    // Configure alert GPIO pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << handle->alert_gpio,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Setup the interrupt service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(handle->alert_gpio, ads1015_isr, handle));

    // Disable interrupt until adc_task is ready to handle it
    gpio_intr_disable(handle->alert_gpio);
    
    xTaskCreate(adc_task, "adc_task", 4096, handle, 5, NULL);

    ads1015_start_conversion(handle);

    ESP_LOGI(TAG, "ADS1015 initialized");

    return ESP_OK;
}

esp_err_t ads1015_start_conversion(ads1015_handle_t *handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return ads1015_write_register(
      handle, 
      ADS1015_CONFIG, 
      handle->config_reg | (1 << 15) // Set OS bit to start conversion
    );
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
