#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"

#include "motorhat.h"
#include "ads1015.h"

#define TAG "MAIN"

void adc_task(void *arg){
    ads1015_handle_t *handle = (ads1015_handle_t *)arg;

    while (1){
        // Wait forever until conversion ready interrupt fires
        if (xSemaphoreTake(handle->rdy_sem, portMAX_DELAY)){
            uint16_t raw;
            ads1015_read_register(handle, ADS1015_CONVERSION, &raw);

            // ADS1015 is 12-bit left aligned
            raw >>= 4;

            ESP_LOGI(TAG, "ADC value: %u", raw);
        }
    }
}

void app_main(void) {

  i2c_bus_t bus;
  i2c_bus_config_t bus_config = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_MOTORHAT_SDA_PIN,
      .scl_io_num = CONFIG_MOTORHAT_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_config));

  ads1015_handle_t ads;
  ads1015_config_t ads_config = {
      .i2c_addr = CONFIG_ADS_ADDRESS,
      .i2c_speed_hz = 400000,
      .alert_gpio = GPIO_NUM_20,
      .bus_handle = bus.handle,
  };

  ESP_ERROR_CHECK(ads_init(&ads, &ads_config));
  xTaskCreate(adc_task, "adc_task", 4096, &ads, 5, NULL);

  // motorhat_handle_t motorhat;

  // motorhat_config_t motorhat_config = {
  //     .pca9685_config =
  //         {
  //             .i2c_addr = CONFIG_MOTORHAT_ADDRESS,
  //             .i2c_speed_hz = 400000,
  //             .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
  //             .bus_handle = bus.handle,
  //         },
  // };

  // ESP_ERROR_CHECK(motorhat_init(&motorhat, &motorhat_config));

  // motorhat_set_motor_direction(&motorhat, MOTORHAT_MOTOR1,
  //                              MOTORHAT_DIRECTION_FORWARD);
  // motorhat_set_motor_speed(&motorhat, MOTORHAT_MOTOR1, 200);

  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // motorhat_set_motor_speed(&motorhat, MOTORHAT_MOTOR1,
  //                          MOTORHAT_DIRECTION_RELEASE);
}
