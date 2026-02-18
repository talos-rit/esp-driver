#include "esp_err.h"

typedef struct {
    int P0_pin ;
    int P1_pin ;
    int glitch_filter_ns ; /**< Pulses shorter than this will not inc/dec the count */
    int resolution ; /**< Number of slots in the encoder wheel, used to calculate the real world angle */ 
    float gear_ratio ; /**< For every X number of rotations of the encoder wheel, the limb it controls moves once */
    float limb_default ; /**< Angle at which the limb hits the end stop switch */
    int invert_angle ; /**< Boolean flip sign of angle and count measurements */
} encoder_config_t ;

typedef struct {
    pcnt_unit_handle_t pcnt_unit ; /**< specific internal PCNT hardware unit  */
} encoder_handle_t ;

/**
 * @brief Initialize an encoder
 *
 * This function initializes the Pulse Counter hardware module for a specific encoder.
 *
 * @param[out] handle Pointer to encoder handle structure
 * @param[in] config Pointer to configuration structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes
 */
esp_err_t encoder_init(encoder_handle_t *handle, encoder_config_t *config) ;

/**
 * @brief Start an encoder
 *
 * Starts the Pulse Counter module for the specified encoder. The count is set to 0.
 *
 * @param[out] handle Pointer to encoder handle structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes
 */
esp_err_t encoder_start(encoder_handle_t *handle) ;

/**
 * @brief Get pulse count of encoder
 *
 * Retrieves the raw pulse count of the encoder. This count is signed, for every slot in 
 * the encoder wheel this count will increment or decrement (based on direction) by 4.
 *
 * @param[out] handle Pointer to encoder handle structure
 * @param[out] pulse_count Pointer to pulse count value desired
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes
 */
esp_err_t encoder_get_raw_count(encoder_handle_t *handle,
                                int *pulse_count) ;

/**
 * @brief Clear pulse count of encoder
 *
 * Resets pulse count of the encoder to 0. This function should only be called when 
 * an end stop has triggered. If called otherwise, all absolute angles will be incorrect.
 *
 * @param[out] handle Pointer to encoder handle structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes
 */
esp_err_t encoder_clear_count(encoder_handle_t *handle) ;

/**
 * @brief Get wheel angle of encoder
 *
 * Retrieves the angle of the encoder wheel. This angle is calculated using the 
 * current pulse count, default pulse count (0), and wheel resolution. This angle is 
 * signed, and can go outside 360.
 *
 * @param[out] handle Pointer to encoder handle structure
 * @param[in] config Pointer to configuration structure
 * @param[out] wheel_angle Pointer to angle value desired
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes
 */
esp_err_t encoder_get_wheel_angle(encoder_handle_t *handle, 
                                    encoder_config_t *config, float *wheel_angle) ;

/**
 * @brief Get limb angle of encoder
 *
 * Retrieves the angle of the encoder wheel. This angle is calculated using the 
 * current pulse count, default pulse count (0), wheel resolution, limb gear ratio, and 
 * default limb angle. This angle is signed, and can go outside 360(though it may 
 * not depending on the robot anatomy).
 *
 * @param[out] handle Pointer to encoder handle structure
 * @param[in] config Pointer to configuration structure
 * @param[out] wheel_angle Pointer to angle value desired
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes
 */                                  
esp_err_t encoder_get_limb_angle(encoder_handle_t *handle,
                                    encoder_config_t *config, float *limb_angle) ;

/**
 * 
 * TODO TODO TODO TODO TODO TODO TODO TODO TODO
 * 
 * @brief register function callback on encoder
 *
 * When the encoder reaches a speicfic pulse count, the function callback defined 
 * here will be executed as an ISR.
 *
 * @param[out] handle Pointer to encoder handle structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes
 */
esp_err_t encoder_register_callback(encoder_handle_t *handle
                                    ) ; //TODO


