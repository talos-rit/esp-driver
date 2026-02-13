#include "ads1015.h"
#include "i2cdev.h"
#include "ads111x.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define ADS1015_I2C_ADDR ADS111X_ADDR_GND   // 0x48
#define ADS1015_I2C_PORT I2C_NUM_0

struct ads1015_handle_t {
    i2c_dev_t dev;
    gpio_num_t alert_gpio;
    SemaphoreHandle_t rdy_sem;
};

esp_err_t ads1015_init(ads1015_handle_t *ads, const ads1015_config_t *config)
{
    if (!ads || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    ads->alert_gpio = config->alert_gpio; 

    ESP_ERROR_CHECK(ads111x_init_desc(
        &ads->dev,
        ADS1015_I2C_ADDR,
        ADS1015_I2C_PORT,
        GPIO_NUM_21,
        GPIO_NUM_22
    ));

    ESP_ERROR_CHECK(ads111x_set_mode(&ads->dev, ADS111X_MODE_CONTINUOUS));
    ESP_ERROR_CHECK(ads111x_set_data_rate(&ads->dev, ADS111X_DATA_RATE_860));
    ESP_ERROR_CHECK(ads111x_set_gain(&ads->dev, ADS111X_GAIN_4V096));

    // configure comparator for conversion ready mode
    ESP_ERROR_CHECK(ads111x_set_comp_mode(&ads->dev, false));                          // coversion ready mode
    ESP_ERROR_CHECK(ads111x_set_comp_polarity(&ads->dev, ADS111X_COMP_POLARITY_LOW));  // active low
    ESP_ERROR_CHECK(ads111x_set_comp_latch(&ads->dev, ADS111X_COMP_LATCH_DISABLED));   // non-latching
    ESP_ERROR_CHECK(ads111x_set_comp_queue(&ads->dev, ADS111X_COMP_QUEUE_1));          // assert after one conversion

    // set high and low threshold registers for RDY mode
    ESP_ERROR_CHECK(ads111x_set_comp_high_thresh(&ads->dev, 0x8000));
    ESP_ERROR_CHECK(ads111x_set_comp_low_thresh(&ads->dev, 0x0000));

    // // Create binary semaphore
    // ads->rdy_sem = xSemaphoreCreateBinary();
    // if (!ads->rdy_sem) {
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

ads1015_handle_t *ads1015_create(void)
{
    return calloc(1, sizeof(ads1015_handle_t));
}

void ads1015_destroy(ads1015_handle_t *ads)
{
    free(ads);
}
