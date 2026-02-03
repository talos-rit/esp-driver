#ifndef _MOTORHAT_H_
#define _MOTORHAT_H_

#include "pca9685.h"
#include <stdint.h>

#define MAX_SPEED 255
static const float DEFAULT_FREQUENCY_HZ = 1526.0f;

typedef enum {
  MOTORHAT_MOTOR1 = 0,
  MOTORHAT_MOTOR2,
  MOTORHAT_MOTOR3,
  MOTORHAT_MOTOR4,
  MOTORHAT_NUM_MOTORS
} motorhat_motor_t;

typedef enum {
  MOTORHAT_DIRECTION_FORWARD = 0,
  MOTORHAT_DIRECTION_BACKWARD,
  MOTORHAT_DIRECTION_BRAKE,
  MOTORHAT_DIRECTION_RELEASE
} motorhat_direction_t;

typedef struct {
  pca9685_channel_t in1_channel;
  pca9685_channel_t in2_channel;
  pca9685_channel_t pwm_channel;
} motorhat_motor_channels_t;

typedef struct {
  pca9685_config_t pca9685_config;
} motorhat_config_t;

typedef struct {
  pca9685_handle_t *pca9685;
} motorhat_handle_t;

static const motorhat_motor_channels_t motor_channels[MOTORHAT_NUM_MOTORS] = {
    {PCA9685_CHANNEL10, PCA9685_CHANNEL9, PCA9685_CHANNEL8},   // Motor 1
    {PCA9685_CHANNEL11, PCA9685_CHANNEL12, PCA9685_CHANNEL13}, // Motor 2
    {PCA9685_CHANNEL4, PCA9685_CHANNEL3, PCA9685_CHANNEL2},    // Motor 3
    {PCA9685_CHANNEL5, PCA9685_CHANNEL6, PCA9685_CHANNEL7}     // Motor 4
};

esp_err_t motorhat_init(motorhat_handle_t *handle,
                        const motorhat_config_t *config);

esp_err_t motorhat_set_motor_speed(motorhat_handle_t *handle,
                                   motorhat_motor_t motor, uint8_t speed);

esp_err_t motorhat_set_motor_direction(motorhat_handle_t *handle,
                                       motorhat_motor_t motor,
                                       motorhat_direction_t direction);

#endif // _MOTORHAT_H_
