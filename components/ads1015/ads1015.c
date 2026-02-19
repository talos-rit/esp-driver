#include "ads1015.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "ADS1015"

esp_err_t ads_init(ads1015_handle_t *handle, const ads1015_config_t *config)
{
    ESP_LOGI(TAG, "Initializing ADS1015...");

    if (!handle || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->i2c_addr,
      .scl_speed_hz = config->i2c_speed_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(config->bus_handle, &dev_config, &handle->dev_handle);
    if (ret != ESP_OK) {
      return ret;
    }

    // Reset the chip
    // ret = pca9685_write_register(handle, PCA9685_MODE1, 0x00);
    // if (ret != ESP_OK) {
    //   return ret;
    // }

    // ESP_ERROR_CHECK(ads111x_set_mode(&ads->dev, ADS111X_MODE_CONTINUOUS));
    // ESP_ERROR_CHECK(ads111x_set_data_rate(&ads->dev, ADS111X_DATA_RATE_860));
    // ESP_ERROR_CHECK(ads111x_set_gain(&ads->dev, ADS111X_GAIN_4V096));

    // // configure comparator for conversion ready mode
    // ESP_ERROR_CHECK(ads111x_set_comp_mode(&ads->dev, false));                          // coversion ready mode
    // ESP_ERROR_CHECK(ads111x_set_comp_polarity(&ads->dev, ADS111X_COMP_POLARITY_LOW));  // active low
    // ESP_ERROR_CHECK(ads111x_set_comp_latch(&ads->dev, ADS111X_COMP_LATCH_DISABLED));   // non-latching
    // ESP_ERROR_CHECK(ads111x_set_comp_queue(&ads->dev, ADS111X_COMP_QUEUE_1));          // assert after one conversion

    // // set high and low threshold registers for RDY mode
    // ESP_ERROR_CHECK(ads111x_set_comp_high_thresh(&ads->dev, 0x8000));
    // ESP_ERROR_CHECK(ads111x_set_comp_low_thresh(&ads->dev, 0x0000));

    // // Create binary semaphore
    // handle->rdy_sem = xSemaphoreCreateBinary();
    // if (!handle->rdy_sem) {
    //     return ESP_ERR_NO_MEM;
    // }

    // // Configure ALERT/RDY pin
    // gpio_config_t io_conf = {
    //     .intr_type = GPIO_INTR_NEGEDGE,   // ADS pulls low when ready
    //     .mode = GPIO_MODE_INPUT,
    //     .pin_bit_mask = 1ULL << ads->alert_gpio,
    //     .pull_up_en = GPIO_PULLUP_ENABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    // };

    // ESP_ERROR_CHECK(gpio_config(&io_conf));

    // // Install ISR service
    // static bool isr_installed = false;
    // if (!isr_installed) {
    //     ESP_ERROR_CHECK(gpio_install_isr_service(0));
    //     isr_installed = true;
    // }

    // ESP_ERROR_CHECK(gpio_isr_handler_add(
    //     ads->alert_gpio,
    //     ads1015_isr,
    //     ads));

    // ESP_LOGI(TAG, "ADS1015 initialized");

    // // configure comparator for conversion ready mode
    // ESP_ERROR_CHECK(ads111x_set_comp_mode(ads->dev, false));     // coversion ready mode
    // ESP_ERROR_CHECK(ads111x_set_comp_polarity(ads->dev, false)); // active low
    // ESP_ERROR_CHECK(ads111x_set_comp_latch(ads->dev, false));    // non-latching
    // ESP_ERROR_CHECK(ads111x_set_comp_queue(ads->dev, 0));        // assert after one conversion

    // set high and low threshold registers for RDY mode
    // ESP_ERROR_CHECK(ads111x_set_comp_high_thresh(ads->dev, 0x8000));
    // ESP_ERROR_CHECK(ads111x_set_comp_low_thresh(ads->dev, 0x0000));

    return ESP_OK;
}
