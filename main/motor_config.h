#ifndef _MOTOR_CONFIG_H_
#define _MOTOR_CONFIG_H_

#include "motorhat.h"
#include "sdkconfig.h"

// The motors in menuconfig's Motors menu. The number is the motor ID that the
// role pickers (Axes menu) and ADC pair pickers resolve to; 0 means none.
// X(id, Kconfig name, display name)
#define MOTOR_CONFIG_COUNT 6
#define MOTOR_CONFIG_LIST(X)      \
  X(1, AXIS1, "Axis 1 (base)")     \
  X(2, AXIS2, "Axis 2 (shoulder)") \
  X(3, AXIS3, "Axis 3 (elbow)")    \
  X(4, AXIS4, "Axis 4 (wrist 4)")  \
  X(5, AXIS5, "Axis 5 (wrist 5)")  \
  X(6, GRIPPER, "Gripper")

// Motor HAT port index of a configured motor: 0-3 = HAT 1 M1-M4,
// 4-7 = HAT 2 M1-M4 (MOTORHAT_MOTOR1-8)
#define MOTOR_CONFIG_PORT_INDEX(SYM)                                    \
  ((CONFIG_DRIVER_MOTOR_##SYM##_HAT - 1) * MOTORHAT_MOTORS_PER_HAT + \
   (CONFIG_DRIVER_MOTOR_##SYM##_PORT - 1))

#endif  // _MOTOR_CONFIG_H_
