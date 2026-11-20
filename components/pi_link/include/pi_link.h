#ifndef _PI_LINK_H_
#define _PI_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// Axis numbers this board supports
#define PI_LINK_AXIS_BASE 1
#define PI_LINK_AXIS_SHOULDER 2
#define PI_LINK_AXIS_ELBOW 3
#define PI_LINK_AXIS_WRIST_4 4  // Wrist motor 4
#define PI_LINK_AXIS_WRIST_5 5  // Wrist motor 5
#define PI_LINK_AXIS_GRIPPER 6
#define PI_LINK_AXIS_SLIDE_BASE 7  // Not built yet
#define PI_LINK_MAX_AXES 7

/**
 * @brief Settings for one axis
 */
typedef struct {
  bool enabled;       /**< If false the axis is never used and reads as 0 */
  int encoder_slots;  /**< Number of slots in this axis's encoder disk */
} pi_link_axis_t;

/**
 * @brief Motion functions the Pi link drives
 */
typedef struct {
  esp_err_t (*polar_pan_start)(int8_t delta_azimuth, int8_t delta_altitude);
  esp_err_t (*polar_pan_stop)(void);  // Also stops the elbow
  esp_err_t (*elbow_start)(int8_t delta_elbow);  // Manual elbow keys, + is '3'
  esp_err_t (*home)(uint16_t delay_ms);  // Returns what motorhat_home returns
  esp_err_t (*home_cancel)(void);  // Makes a running home() return early
  esp_err_t (*set_speed_percent)(uint8_t percent);  // SPEED n, 1 to 100
  // Reads the encoder count of an enabled axis
  esp_err_t (*read_axis_count)(int axis, int32_t* count);
} pi_link_motor_interface_t;

/**
 * @brief Pi link configuration structure
 */
typedef struct {
  int baud_rate;    /**< Must match the Pi's tty */
  bool status_lines; /**< Send "ESP ..." status lines, which operator only logs */
  pi_link_motor_interface_t motor; /**< Motion functions, all required */
  pi_link_axis_t axes[PI_LINK_MAX_AXES + 1]; /**< Indexed by axis number so [0] is unused */
} pi_link_config_t;

/**
 * @brief Take over UART0 and start reading ACL from the Pi's operator (erv)
 *
 * @param[in] config Pointer to configuration structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument
 *    - ESP_ERR_NO_MEM: Task could not be created
 *    - ESP_ERR_*: Errors from the UART driver
 */
esp_err_t pi_link_init(const pi_link_config_t* config);

/**
 * @brief Send a status line to the Pi, if status lines are on
 *
 * Start it with "ESP " so operator only logs it. Do not put '>' in it since
 * operator reads that as the prompt.
 *
 * @param[in] text The line without \r\n
 */
void pi_link_send_status(const char* text);

#endif  // _PI_LINK_H_
