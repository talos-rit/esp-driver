#include "esp_err.h"

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#define ENCODER_LOW_LIMIT INT32_MIN
#define ENCODER_HIGH_LIMIT INT32_MAX

typedef struct {
    int P0_pin ;
    int P1_pin ;
    int glitch_filter_ns ; /**< Pulses shorter than this will not inc/dec the count */
    int resolution ; /**< Number of slots in the encoder wheel, used to calculate the real world angle */ 
    float gear_ratio ; /**< For every X number of rotations of the encoder wheel, the limb it controls moves once */
    float limb_default ; /**< Angle at which the limb hits the end stop switch */
    int invert_angle ; /**< Boolean flip sign of angle and count measurements */
    int lim_low ;
    int lim_high ;
} encoder_config_t ;

typedef struct {
    pcnt_unit_handle_t pcnt_unit ; /**< specific internal PCNT hardware unit  */
} encoder_handle_t ;

typedef enum {
    STATIC_WATCH_POINT, /**< Events are called when the raw count reaches this point */
    INCREMENTAL_WATCH_POINT /**< Events are called when the raw count increments by this amount every time */
} encoder_watch_point_t ;

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
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
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
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
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
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
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
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
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
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
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
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *    - ESP_ERR_*: Other ESP-IDF error codes
 */                                  
esp_err_t encoder_get_limb_angle(encoder_handle_t *handle,
                                    encoder_config_t *config, float *limb_angle) ;

// /**
//  * 
//  * TODO TODO TODO TODO TODO TODO TODO TODO TODO
//  * 
//  * @brief register function callback on encoder
//  *
//  * Callbacks are ran in an ISR context. All callbacks are called at EVERY watch point. 
//  * Watch points must be set using the encoder_register_watchpoint() function. This function must be called while the encoder is disabled.
//  *
//  * @param[out] handle Pointer to encoder handle structure
//  * @param[in] callback Function point to be called
//  *
//  * @return
//  *    - ESP_OK: Success
//  *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
//  *    - ESP_ERR_*: Other ESP-IDF error codes
//  */
// esp_err_t encoder_register_callback(encoder_handle_t *handle,
//                                     void* callback) ;
// esp_err_t encoder_deregister_callback(encoder_handle_t *handle,
//                                     void* callback) ;

// esp_err_t encoder_register_watchpoint(encoder_handle_t *handle,
//                                     encoder_watch_point_t type, int watch_point) ;
// esp_err_t encoder_deregister_watchpoint(encoder_handle_t *handle,
//                                     encoder_watch_point_t type, int watch_point) ;

