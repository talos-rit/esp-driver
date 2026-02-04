#include "pca9685.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <stdint.h>

static pca9685_channel_registers_t channel_regs[PCA9685_CHANNEL15 + 1] = {
    {PCA9685_LED0_ON_L, PCA9685_LED0_ON_H, PCA9685_LED0_OFF_L,
     PCA9685_LED0_OFF_H},
    {PCA9685_LED1_ON_L, PCA9685_LED1_ON_H, PCA9685_LED1_OFF_L,
     PCA9685_LED1_OFF_H},
    {PCA9685_LED2_ON_L, PCA9685_LED2_ON_H, PCA9685_LED2_OFF_L,
     PCA9685_LED2_OFF_H},
    {PCA9685_LED3_ON_L, PCA9685_LED3_ON_H, PCA9685_LED3_OFF_L,
     PCA9685_LED3_OFF_H},
    {PCA9685_LED4_ON_L, PCA9685_LED4_ON_H, PCA9685_LED4_OFF_L,
     PCA9685_LED4_OFF_H},
    {PCA9685_LED5_ON_L, PCA9685_LED5_ON_H, PCA9685_LED5_OFF_L,
     PCA9685_LED5_OFF_H},
    {PCA9685_LED6_ON_L, PCA9685_LED6_ON_H, PCA9685_LED6_OFF_L,
     PCA9685_LED6_OFF_H},
    {PCA9685_LED7_ON_L, PCA9685_LED7_ON_H, PCA9685_LED7_OFF_L,
     PCA9685_LED7_OFF_H},
    {PCA9685_LED8_ON_L, PCA9685_LED8_ON_H, PCA9685_LED8_OFF_L,
     PCA9685_LED8_OFF_H},
    {PCA9685_LED9_ON_L, PCA9685_LED9_ON_H, PCA9685_LED9_OFF_L,
     PCA9685_LED9_OFF_H},
    {PCA9685_LED10_ON_L, PCA9685_LED10_ON_H, PCA9685_LED10_OFF_L,
     PCA9685_LED10_OFF_H},
    {PCA9685_LED11_ON_L, PCA9685_LED11_ON_H, PCA9685_LED11_OFF_L,
     PCA9685_LED11_OFF_H},
    {PCA9685_LED12_ON_L, PCA9685_LED12_ON_H, PCA9685_LED12_OFF_L,
     PCA9685_LED12_OFF_H},
    {PCA9685_LED13_ON_L, PCA9685_LED13_ON_H, PCA9685_LED13_OFF_L,
     PCA9685_LED13_OFF_H},
    {PCA9685_LED14_ON_L, PCA9685_LED14_ON_H, PCA9685_LED14_OFF_L,
     PCA9685_LED14_OFF_H},
    {PCA9685_LED15_ON_L, PCA9685_LED15_ON_H, PCA9685_LED15_OFF_L,
     PCA9685_LED15_OFF_H},
};

esp_err_t pca9685_init(pca9685_handle_t *handle,
                       const pca9685_config_t *config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

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

  ret = pca9685_write_register(handle, PCA9685_MODE1, 0x00);
  if (ret != ESP_OK) {
    return ret;
  }

  if (config->pwm_freq_hz < 24.0f || config->pwm_freq_hz > 1526.0f) {
    return ESP_ERR_INVALID_ARG;
  }

  float prescaleval = 25000000.0f; // 25MHz
  prescaleval /= 4096.0f;          // 12-bit
  prescaleval /= config->pwm_freq_hz;
  prescaleval -= 1.0f;

  uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

  uint8_t oldmode;
  ret = pca9685_read_register(handle, PCA9685_MODE1, &oldmode, 1);
  if (ret != ESP_OK) {
    return ret;
  }

  uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
  ret = pca9685_write_register(handle, PCA9685_MODE1, newmode);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = pca9685_write_register(handle, PCA9685_PRE_SCALE, prescale);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = pca9685_write_register(handle, PCA9685_MODE1, oldmode);
  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(5));

  // Enable auto increment
  ret = pca9685_write_register(handle, PCA9685_MODE1, oldmode | 0xA0);
  if (ret != ESP_OK) {
    return ret;
  }

  return ESP_OK;
}

esp_err_t pca9685_set_duty_cycle(pca9685_handle_t *handle,
                                 pca9685_channel_t channel, float duty_cycle) {
  if (handle == NULL || channel < PCA9685_CHANNEL0 ||
      channel > PCA9685_CHANNEL15) {
    return ESP_ERR_INVALID_ARG;
  }

  if (duty_cycle < 0.0f || duty_cycle > 1.0f) {
    return ESP_ERR_INVALID_ARG;
  }

  uint16_t on = 0;
  uint16_t off = (uint16_t)(duty_cycle * PCA9685_PWM_MAX);

  // Handle special cases for full on and full off
  if (duty_cycle == 1.0f) {
    on = PCA9685_PWM_MAX;
    off = 0;
  } else if (duty_cycle == 0.0f) {
    on = 0;
    off = PCA9685_PWM_MAX;
  }

  return pca9685_write_channel_registers(handle, channel, on, off);
}

esp_err_t pca9685_digital_write(pca9685_handle_t *handle,
                                pca9685_channel_t channel, bool level) {
  if (handle == NULL || channel < PCA9685_CHANNEL0 ||
      channel > PCA9685_CHANNEL15) {
    return ESP_ERR_INVALID_ARG;
  }

  if (level) {
    return pca9685_write_channel_registers(handle, channel, PCA9685_PWM_MAX, 0);
  } else {
    return pca9685_write_channel_registers(handle, channel, 0, PCA9685_PWM_MAX);
  }
}

esp_err_t pca9685_write_channel_registers(pca9685_handle_t *handle,
                                          pca9685_channel_t channel,
                                          uint16_t on, uint16_t off) {
  if (handle == NULL || channel < PCA9685_CHANNEL0 ||
      channel > PCA9685_CHANNEL15) {
    return ESP_ERR_INVALID_ARG;
  }

  if (on > PCA9685_PWM_MAX || off > PCA9685_PWM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  pca9685_channel_registers_t regs = channel_regs[channel];

  uint8_t write_buf[5];
  write_buf[0] = regs.on_low;
  write_buf[1] = on & 0xFF;
  write_buf[2] = (on >> 8) & 0x1F;
  write_buf[3] = off & 0xFF;
  write_buf[4] = (off >> 8) & 0x1F;

  return i2c_master_transmit(handle->dev_handle, write_buf, sizeof(write_buf),
                             PCA9685_I2C_MASTER_TIMEOUT_MS);
}

esp_err_t pca9685_read_register(pca9685_handle_t *handle,
                                pca9685_register_t reg, uint8_t *data,
                                size_t len) {
  if (handle == NULL || data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg_buf[1] = {reg};

  return i2c_master_transmit_receive(handle->dev_handle, reg_buf, sizeof(reg),
                                     data, len, PCA9685_I2C_MASTER_TIMEOUT_MS);
}

esp_err_t pca9685_write_register(pca9685_handle_t *handle,
                                 pca9685_register_t reg, uint8_t data) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t write_buf[2] = {reg, data};
  return i2c_master_transmit(handle->dev_handle, write_buf, sizeof(write_buf),
                             PCA9685_I2C_MASTER_TIMEOUT_MS);
}
