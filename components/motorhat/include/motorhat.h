#ifndef _MOTORHAT_H_
#define _MOTORHAT_H_

#include "pca9685.h"
#include <stdint.h>

static const float DEFAULT_FREQUENCY_HZ = 1526.0f;

typedef enum {
  MOTORHAT_MOTOR1 = 0,
  MOTORHAT_MOTOR2,
  MOTORHAT_MOTOR3,
  MOTORHAT_MOTOR4,
  MOTORHAT_NUM_MOTORS
} motorhat_motor_t;

/**
 * @brief Motor direction and state control
 */
typedef enum {
  MOTORHAT_DIRECTION_FORWARD = 0,  /**< Motor rotates forward */
  MOTORHAT_DIRECTION_BACKWARD,     /**< Motor rotates backward */
  MOTORHAT_DIRECTION_BRAKE,        /**< Motor brakes (both inputs high) */
  MOTORHAT_DIRECTION_RELEASE       /**< Motor released/coasting (both inputs low) */
} motorhat_direction_t;

/**
 * @brief Channel mapping for a single motor
 * 
 * Maps the three PCA9685 channels required to control one motor:
 * - IN1 and IN2: Direction control inputs to the H-bridge
 * - PWM: Speed control via pulse-width modulation
 */
typedef struct {
  pca9685_channel_t in1_channel;  /**< H-bridge IN1 control channel */
  pca9685_channel_t in2_channel;  /**< H-bridge IN2 control channel */
  pca9685_channel_t pwm_channel;  /**< PWM speed control channel */
} motorhat_motor_channels_t;

/**
 * @brief Motor HAT configuration structure
 */
typedef struct {
  pca9685_config_t pca9685_config;
} motorhat_config_t;

/**
 * @brief Motor HAT device handle
 */
typedef struct {
  pca9685_handle_t *pca9685;
} motorhat_handle_t;

/**
 * @brief Channel assignments for each motor on the Adafruit Motor HAT
 * 
 * This mapping is specific to the Adafruit Motor HAT v2 pinout:
 * - Motor 1: IN1=CH10, IN2=CH9,  PWM=CH8
 * - Motor 2: IN1=CH11, IN2=CH12, PWM=CH13
 * - Motor 3: IN1=CH4,  IN2=CH3,  PWM=CH2
 * - Motor 4: IN1=CH5,  IN2=CH6,  PWM=CH7
 */
static const motorhat_motor_channels_t motor_channels[MOTORHAT_NUM_MOTORS] = {
    {PCA9685_CHANNEL10, PCA9685_CHANNEL9, PCA9685_CHANNEL8},   // Motor 1
    {PCA9685_CHANNEL11, PCA9685_CHANNEL12, PCA9685_CHANNEL13}, // Motor 2
    {PCA9685_CHANNEL4, PCA9685_CHANNEL3, PCA9685_CHANNEL2},    // Motor 3
    {PCA9685_CHANNEL5, PCA9685_CHANNEL6, PCA9685_CHANNEL7}     // Motor 4
};

/**
 * @brief Initialize the Motor HAT controller
 *
 * Initializes the underlying PCA9685 PWM controller with the provided configuration.
 * After successful initialization, motors can be controlled using the speed and
 * direction functions.
 *
 * @param[out] handle Pointer to Motor HAT handle structure
 * @param[in] config Pointer to configuration structure containing PCA9685 settings
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *    - ESP_ERR_*: Other ESP-IDF error codes from PCA9685 initialization
 *
 * @note The handle->pca9685 pointer must be allocated and point to a valid
 *       pca9685_handle_t structure before calling this function
 */
esp_err_t motorhat_init(motorhat_handle_t *handle,
                        const motorhat_config_t *config);

/**
 * @brief Set motor speed
 *
 * Sets the PWM duty cycle controlling motor speed. The speed does not affect
 * direction - use motorhat_set_motor_direction() to control direction.
 *
 * @param[in] handle Pointer to Motor HAT handle
 * @param[in] motor Motor identifier (MOTORHAT_MOTOR1 through MOTORHAT_MOTOR4)
 * @param[in] speed Speed value (0-4096, where 0=stopped, 4096=full speed)
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle, invalid motor, or speed > 4096)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 *
 * @note For typical usage with 0-255 speed range, scale your value:
 *       actual_speed = (your_speed * PCA9685_PWM_MAX) / 255
 */
esp_err_t motorhat_set_motor_speed(motorhat_handle_t *handle,
                                   motorhat_motor_t motor, uint16_t speed);

/**
 * @brief Set motor direction
 *
 * Controls the motor's direction and braking state by setting the IN1 and IN2
 * control signals appropriately:
 * - FORWARD: IN1=HIGH, IN2=LOW
 * - BACKWARD: IN1=LOW, IN2=HIGH  
 * - BRAKE: IN1=HIGH, IN2=HIGH (active braking)
 * - RELEASE: IN1=LOW, IN2=LOW (coasting/free-running)
 *
 * @param[in] handle Pointer to Motor HAT handle
 * @param[in] motor Motor identifier (MOTORHAT_MOTOR1 through MOTORHAT_MOTOR4)
 * @param[in] direction Desired direction/state
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle, invalid motor, or invalid direction)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 *
 * @note Setting direction does not affect speed. Set speed separately using
 *       motorhat_set_motor_speed()
 */
esp_err_t motorhat_set_motor_direction(motorhat_handle_t *handle,
                                       motorhat_motor_t motor,
                                       motorhat_direction_t direction);

#endif // _MOTORHAT_H_
