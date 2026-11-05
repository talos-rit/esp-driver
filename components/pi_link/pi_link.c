#include "pi_link.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/uart_pins.h"

#define TAG "pi_link"

#define PI_LINK_UART UART_NUM_0
#define PI_LINK_RX_BUFFER_SIZE 256  // Driver needs more than the 128 byte FIFO
#define PI_LINK_LINE_MAX 64         // ACL_CMD_SIZE in operator acl.hpp
#define PI_LINK_READ_TIMEOUT_MS 10  // erv sends manual keys every 10 ms
#define PI_LINK_KEY_TIMEOUT_MS 100  // Stop if erv's 10 ms key repeat stops
#define PI_LINK_RESTART_MS 250  // Repeat start below CONFIG_DRIVER_MAX_RUN_MS

#define ACL_MANUAL_TOGGLE '~'
#define ACL_XON 0x11   // The Pi's tty uses XON/XOFF flow control (ixon ixoff)
#define ACL_XOFF 0x13
#define ACL_CTRL_A 0x01  // Abort movement
#define ACL_CTRL_C 0x03  // Cut off a listing, sent by the dev erv

static pi_link_config_t s_config;
static bool s_manual_mode = false;
static char s_line[PI_LINK_LINE_MAX];
static size_t s_line_len = 0;
static bool s_line_bad = false;
static uint8_t s_active_key = 0;  // Key driving the motors, 0 when stopped
static TickType_t s_last_key_tick;
static TickType_t s_last_start_tick;
static volatile bool s_homing = false;  // Cleared by home_task when done

static void reply(const char* text) {
  if (!s_config.debug_reply) return;
  uart_write_bytes(PI_LINK_UART, text, strlen(text));
}

// Manual mode keys erv can send (ACL_Manual_Axis in operator acl_private.hpp).
// Which key of each pair is + differs between operator versions; the build on
// the Pi image is the reference, so check directions on the bench
static bool is_manual_key(uint8_t byte) {
  switch (byte) {
    case '1':
    case 'q':  // base
    case '2':
    case 'w':  // shoulder
    case '3':
    case 'e':  // elbow
    case '4':
    case 'r':  // wrist pitch
    case '5':
    case 't':  // wrist roll
      return true;
    default:
      return false;
  }
}

// Checked on the bench against the Pi image's erv:
// RIGHT 'q', LEFT '1', UP '4', DOWN 'r'
static bool key_to_deltas(uint8_t key, int8_t* azimuth, int8_t* altitude) {
  *azimuth = 0;
  *altitude = 0;
  switch (key) {
    case 'q':
      *azimuth = 1;
      return true;
    case '1':
      *azimuth = -1;
      return true;
    case '4':
      *altitude = 1;
      return true;
    case 'r':
      *altitude = -1;
      return true;
    default:
      return false;  // Shoulder, elbow and roll are not driven
  }
}

static void stop_motion(void) {
  s_active_key = 0;
  // Homing stops the motors itself, and a stop landing as it starts could
  // brake the axis it is driving
  if (s_homing) return;
  s_config.motor.polar_pan_stop();
}

static void handle_manual_toggle(void) {
  // Nothing should be moving on entering or leaving manual mode
  stop_motion();
  reply(s_manual_mode ? "manual on\r\n" : "manual off\r\n");
}

static void handle_manual_key(uint8_t key) {
  char text[16];
  snprintf(text, sizeof(text), "key %c\r\n", key);
  reply(text);

  int8_t azimuth, altitude;
  if (!key_to_deltas(key, &azimuth, &altitude)) {
    if (s_active_key) stop_motion();
    return;
  }
  if (s_homing) return;

  TickType_t now = xTaskGetTickCount();
  s_last_key_tick = now;

  // Start on a new key, and repeat it while the key is held: each start
  // renews the max run check and re-checks the soft limits
  if (key != s_active_key ||
      now - s_last_start_tick >= pdMS_TO_TICKS(PI_LINK_RESTART_MS)) {
    s_last_start_tick = now;
    if (s_config.motor.polar_pan_start(azimuth, altitude) == ESP_OK) {
      s_active_key = key;
    } 
    else {
      s_active_key = 0;  // Retried on the next key
    }
  }
}

// Called every read, so a key stream that stops also stops the motors
static void check_key_timeout(void) {
  if (s_active_key && xTaskGetTickCount() - s_last_key_tick > pdMS_TO_TICKS(PI_LINK_KEY_TIMEOUT_MS)) {
    stop_motion();
    reply("key timeout\r\n");
  }
}

// motorhat_home blocks until done, so it runs here to keep the link read
static void home_task(void* arg) {
  s_config.motor.home(0);
  s_homing = false;
  vTaskDelete(NULL);
}

static void start_homing(void) {
  if (s_homing) return;
  stop_motion();
  s_homing = true;
  if (xTaskCreate(home_task, "pi_link_home", 4096, NULL, 5, NULL) != pdPASS) {
    s_homing = false;
  }
}

// Sent after each command, see prompt() below
static void prompt(void);

// ---------------------------------------------------------------------------
// Line commands
//
// Every ACL line the operator (erv) can send, plus the ESP native commands
// from the Pi compatibility spec. Matching is case insensitive, like the ER-V
// controller (erv sends both "HOME" and "clrbuf"). Each handler gets the text
// after the command name, "" when there is none.
//
// Sent by the old erv (operator main, the Pi image's operator2 bb8dbe4):
//   \r, ~, manual keys, HERE DELTA, SHIFT DELTA BY a n, clrbuf, MOVE DELTA,
//   DEFP DELTA, HOME
// Also sent by the dev erv (operator twinning 5a18ea6):
//   CON, SPEED n, LISTPV POSITION, Ctrl-C, and it reads the '>' prompt
// ---------------------------------------------------------------------------

// Rejects anything after a command that takes no arguments, so a corrupted
// line is not run as something else
static bool no_args(const char* cmd, const char* args) {
  if (*args == '\0') return true;
  char text[32];
  snprintf(text, sizeof(text), "%s bad argument\r\n", cmd);
  reply(text);
  return false;
}

// Parses a whole decimal integer. Trailing text fails, so "5x" is not 5
static bool parse_long(const char* text, long* value) {
  char* end;
  *value = strtol(text, &end, 10);
  return end != text && *end == '\0';
}

// HOME: home the arm. erv also sends it once at startup (DEFP, HOME, HERE).
// TODO: the controller also takes HOME n to home one axis. Homing one axis is
// not supported, so HOME n homes everything
static void cmd_home(const char* args) {
  (void)args;
  start_homing();
}

// clrbuf: empty the movement buffer and halt all motors. erv sends it as its
// stop and before every MOVE.
// TODO: must also cancel homing (spec priority 1). motorhat_home has no cancel
// or timeout, so homing currently runs to the end
static void cmd_clrbuf(const char* args) {
  (void)args;  // clrbuf axis would clear one axis; stopping all is safer
  stop_motion();
}

// A: abort the current movement (Ctrl-A is handled in handle_byte). Not sent
// by erv. The controller keeps its movement buffer; this board has none, so
// this is the same as clrbuf.
// TODO: cancel homing too, same as clrbuf
static void cmd_abort(const char* args) {
  (void)args;
  stop_motion();
}

// STOP: ESP native stop, same as clrbuf.
// TODO: cancel homing too, same as clrbuf
static void cmd_stop(const char* args) {
  (void)args;
  stop_motion();
}

// SPEED n: n 1 to 100, a percent of the configured pan speed (sent by
// Commander's speed control through the dev erv)
static void cmd_speed(const char* args) {
  long percent;
  if (!parse_long(args, &percent) || percent < 1 || percent > 100) {
    reply("speed bad argument\r\n");
    return;
  }

  if (s_config.motor.set_speed_percent((uint8_t)percent) != ESP_OK) {
    reply("speed failed\r\n");
    return;
  }

  char text[24];
  snprintf(text, sizeof(text), "speed %ld\r\n", percent);
  reply(text);
}

// CON: enable servo control. The dev erv sends it for Commander's enable
// operation (ICD hardware op 0x04) to recover from CONTROL DISABLED.
// TODO: the motors have no enable state yet, so this does nothing. Once COFF
// and fault latching exist (overcurrent, impact, limit), CON should clear the
// latch and allow motion again
static void cmd_con(const char* args) {
  (void)args;  // CON axis would enable one axis; treated like CON
  reply("con: nothing to enable\r\n");
}

// COFF: disable servo control. Not sent by erv. Stops the motors so it is at
// least safe.
// TODO: latch a disabled state that refuses manual keys, HOME and MOVE until
// CON, and report CONTROL DISABLED like the controller does
static void cmd_coff(const char* args) {
  (void)args;
  stop_motion();
  reply("coff: stopped, not latched\r\n");
}

// DEFP pos: define a position variable. erv only uses DELTA (home sequence).
// TODO: store a named position (DELTA alone is enough for erv) once positions
// and relative moves exist
static void cmd_defp(const char* args) {
  (void)args;
  reply("defp: not implemented\r\n");
}

// HERE pos: record the current position into pos. erv sends HERE DELTA before
// every relative move and 500 ms after each stop.
// TODO: copy each axis's current encoder count into DELTA, the base that SHIFT
// adds to and MOVE drives to
static void cmd_here(const char* args) {
  (void)args;
  reply("here: not implemented\r\n");
}

// SHIFT pos BY axis counts: add counts to one axis of pos. erv only sends
// "SHIFT DELTA BY <axis> <counts>", axes 1 base, 2 shoulder, 3 elbow,
// 4 wrist pitch.
// TODO: add counts to that axis of DELTA. The counts are ER-V encoder counts
// (base 42.5666/deg, shoulder 33.2121/deg, wrist 8.3555/deg in operator
// acl_private.hpp), so convert to this board's counts per degree (see
// compute_soft_limit in esp_driver.c), base -> azimuth and wrist pitch ->
// altitude. Clamp to the soft limits and refuse shoulder and elbow
static void cmd_shift(const char* args) {
  char pos[16];
  int axis;
  long counts;
  int used = 0;
  if (sscanf(args, "%15s BY %d %ld %n", pos, &axis, &counts, &used) != 3 ||
      args[used] != '\0') {
    reply("shift bad argument\r\n");
    return;
  }

  char text[64];
  snprintf(text, sizeof(text), "shift %s axis %d by %ld: not implemented\r\n",
           pos, axis, counts);
  reply(text);
}

// MOVE pos [duration] / MOVED pos [duration]: drive to pos. erv sends
// MOVE DELTA after its SHIFTs. MOVED also holds the next command until the
// move ends, which only matters inside controller programs.
// TODO: needs a position move in motorhat (motorhat_polar_pan is still a
// TODO). Drive each axis toward DELTA under the soft limits and max run time,
// with duration (hundredths of a second) setting the speed when given.
// clrbuf, A and STOP must cancel it
static void cmd_move(const char* args) {
  (void)args;
  reply("move: not implemented\r\n");
}

// LISTPV POSITION: list the arm's joint encoder counts. The dev erv polls it
// for telemetry about once a second when idle. The controller replies
//   Position POSITION
//    1:<base> 2:<shoulder> 3:<elbow> 4:<wrist pitch> 5:<wrist roll>
//   X:... Y:... Z:... P:... R:...
// and erv only parses the joint line (" %*d:%ld" five times).
// TODO: needs an encoder read in pi_link_motor_interface_t. Reply with
// azimuth as axis 1 and altitude as axis 4 in ER-V counts, 0 for the others,
// and skip the Cartesian line. Only POSITION needs supporting
static void cmd_listpv(const char* args) {
  (void)args;
  reply("listpv: not implemented\r\n");
}

// VERSION: ESP native, firmware version.
// TODO: reply "ESP VERSION <version>" from esp_app_get_description()
static void cmd_version(const char* args) {
  if (!no_args("VERSION", args)) return;
  reply("version: not implemented\r\n");
}

// CAPS: ESP native, which commands and axes this board supports.
// TODO: reply e.g. "ESP CAPS AXES=AZIMUTH,ALTITUDE HOME SPEED STOP" built from
// the menuconfig settings (homing, soft limits, second altitude motor)
static void cmd_caps(const char* args) {
  if (!no_args("CAPS", args)) return;
  reply("caps: not implemented\r\n");
}

// STATUS: ESP native, current state.
// TODO: reply with manual mode, homing, homed, moving axes, speed percent and
// any fault (estop, overcurrent, limit)
static void cmd_status(const char* args) {
  if (!no_args("STATUS", args)) return;
  reply("status: not implemented\r\n");
}

// ENCODER: ESP native, raw encoder counts.
// TODO: needs the same encoder read as LISTPV. Reply
// "ESP ENCODER AZ=<count> ALT=<count>" in this board's counts
static void cmd_encoder(const char* args) {
  if (!no_args("ENCODER", args)) return;
  reply("encoder: not implemented\r\n");
}

typedef struct {
  const char* name;
  void (*handler)(const char* args);
} line_command_t;

static const line_command_t s_line_commands[] = {
    // Implemented
    {"HOME", cmd_home},
    {"clrbuf", cmd_clrbuf},
    {"A", cmd_abort},
    {"STOP", cmd_stop},
    {"SPEED", cmd_speed},
    // Stubs, see the TODO on each handler
    {"CON", cmd_con},
    {"COFF", cmd_coff},
    {"DEFP", cmd_defp},
    {"HERE", cmd_here},
    {"SHIFT", cmd_shift},
    {"MOVE", cmd_move},
    {"MOVED", cmd_move},
    {"LISTPV", cmd_listpv},
    {"VERSION", cmd_version},
    {"CAPS", cmd_caps},
    {"STATUS", cmd_status},
    {"ENCODER", cmd_encoder},
};

static void handle_line(const char* line) {
  char text[PI_LINK_LINE_MAX + 16];
  snprintf(text, sizeof(text), "line [%s]\r\n", line);
  reply(text);

  // "NAME args": the name ends at the first space
  const char* space = strchr(line, ' ');
  size_t name_len = space ? (size_t)(space - line) : strlen(line);
  const char* args = space ? space + 1 : "";
  while (*args == ' ') args++;

  for (size_t i = 0; i < sizeof(s_line_commands) / sizeof(s_line_commands[0]);
       i++) {
    const line_command_t* cmd = &s_line_commands[i];
    if (strlen(cmd->name) == name_len &&
        strncasecmp(line, cmd->name, name_len) == 0) {
      cmd->handler(args);
      prompt();
      return;
    }
  }

  // TODO: the controller replies "ACL - unrecognized request", which the dev
  // erv reports as a FAULT; the ESP spec wants "ESP ERROR UNSUPPORTED <cmd>".
  // Pick one once replies go out on the wire
  reply("unknown command\r\n");
  prompt();
}

// The controller prints a '>' prompt when it is ready for the next command.
// The old erv ignores it, but the dev erv (5a18ea6) depends on it: '>' sets
// its direct_mode, and without direct_mode it never sends '~' or manual keys
// after a Ctrl-C, and never polls LISTPV.
// TODO: write ">" here once the wire protocol is settled. The ESP spec says no
// prompt and no output on UART0, and the Pi tty's echo setting is unverified,
// so nothing is sent yet
static void prompt(void) {}

static void reset_line(void) {
  s_line_len = 0;
  s_line_bad = false;
}

static void handle_byte(uint8_t byte) {
  // Flow control from the Pi, not part of any command
  if (byte == ACL_XON || byte == ACL_XOFF) return;

  // Ctrl-C: the dev erv sends it to cut off a LISTPV listing before it starts
  // a manual pan, then waits for the prompt. Nothing here prints listings, so
  // drop any partial line and prompt. Motion is not affected, as on the
  // controller
  if (byte == ACL_CTRL_C) {
    reset_line();
    prompt();
    return;
  }

  // Ctrl-A: abort movement at once, the same as the A command
  if (byte == ACL_CTRL_A) {
    reset_line();
    stop_motion();
    prompt();
    return;
  }

  if (byte == ACL_MANUAL_TOGGLE) {
    reset_line();  // erv only sends ~ between whole commands
    s_manual_mode = !s_manual_mode;
    handle_manual_toggle();
    return;
  }

  if (s_manual_mode) {
    if (is_manual_key(byte)) {
      handle_manual_key(byte);
      return;
    }
    // In manual mode erv only sends keys and ~. Anything else means a ~ was
    // missed (e.g. this board rebooted mid pan), so leave manual mode and
    // read the byte as the start of a command line
    s_manual_mode = false;
    handle_manual_toggle();
  }

  if (byte == '\r' || byte == '\n') {
    if (s_line_len > 0 && !s_line_bad) {
      s_line[s_line_len] = '\0';
      handle_line(s_line);
    }
    reset_line();
    return;
  }

  if (s_line_bad) return;
  // Drop the whole line rather than run part of a corrupted or long one
  if (!isprint(byte) || s_line_len >= PI_LINK_LINE_MAX - 1) {
    s_line_bad = true;
    return;
  }
  s_line[s_line_len++] = (char)byte;
}

static void pi_link_task(void* arg) {
  uint8_t rx[32];

  while (1) {
    int len = uart_read_bytes(PI_LINK_UART, rx, sizeof(rx), pdMS_TO_TICKS(PI_LINK_READ_TIMEOUT_MS));
    if (len < 0) {
      ESP_LOGE(TAG, "UART read failed");
      len = 0;  // Still run the key timeout below
    }

    for (int i = 0; i < len; i++) {
      handle_byte(rx[i]);
    }
    check_key_timeout();
  }
}

esp_err_t pi_link_init(const pi_link_config_t* config) {
  if (!config || !config->motor.polar_pan_start ||
      !config->motor.polar_pan_stop || !config->motor.home ||
      !config->motor.set_speed_percent) {
    return ESP_ERR_INVALID_ARG;
  }
  s_config = *config;

  uart_config_t uart_config = {
      .baud_rate = config->baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t err = uart_driver_install(PI_LINK_UART, PI_LINK_RX_BUFFER_SIZE, 0, 0, NULL, 0);
  if (err != ESP_OK) return err;

  err = uart_param_config(PI_LINK_UART, &uart_config);
  if (err != ESP_OK) return err;

  // With console output set to None the bootloader detaches UART0 from its
  // pins, so connect it back to GPIO1/3, which the dev board wires to USB
  err = uart_set_pin(PI_LINK_UART, U0TXD_GPIO_NUM, U0RXD_GPIO_NUM,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) return err;

  // Drop anything that arrived while booting
  err = uart_flush_input(PI_LINK_UART);
  if (err != ESP_OK) return err;

  // The ROM boot message goes out at 115200 and reads as random bytes at
  // 9600. One of them can look like XOFF and pause everything the Pi sends,
  // stops included, so tell the Pi to resume
  const char xon = ACL_XON;
  if (uart_write_bytes(PI_LINK_UART, &xon, 1) < 0) return ESP_FAIL;

  if (xTaskCreate(pi_link_task, "pi_link_task", 4096, NULL, 5, NULL) !=
      pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}
