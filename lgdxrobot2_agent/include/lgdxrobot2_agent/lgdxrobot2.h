#ifndef __LGDXROBOT2_H
#define __LGDXROBOT2_H

#include <stdint.h>
#include <stdbool.h>

// Operation Configuration
#define PID_LEVEL 3
#define API_MOTOR_COUNT 4

#define MCU_HEADER1 0xAA
#define MCU_HEADER2 0x55
#define MCU_HEADER3 0xA5
#define MCU_HEADER4 0x5A

#define MCU_DATA_TYPE 'D'
#define MCU_SERIAL_NUMBER_TYPE 'S'
#define MCU_PID_TYPE 'P'

#define MCU_SOFTWARE_EMERGENCY_STOP_COMMAND_TYPE 'E'
#define MCU_INVERSE_KINEMATICS_COMMAND_TYPE 'I'
#define MCU_MOTOR_COMMAND_TYPE 'M'
#define MCU_SET_PID_SPEED_COMMAND_TYPE 'L'
#define MCU_GET_PID_COMMAND_TYPE 'P'
#define MCU_SET_PID_COMMAND_TYPE 'Q'
#define MCU_SAVE_PID_COMMAND_TYPE 'R'
#define MCU_GET_SERIAL_NUMBER_COMMAND_TYPE 'S'
#define MCU_RESET_TRANSFORM_COMMAND_TYPE 'T'
#define MCU_SET_MOTOR_MAXIMUM_SPEED_COMMAND_TYPE 'V'

#pragma pack(push, 1)

/*
 * MCU to PC communication
 */

typedef struct {
  float x;
  float y;
  float rotation;
} McuDof;

typedef struct {
  float voltage;
  float current;
} McuPower;

typedef struct {
  uint8_t header1;
  uint8_t header2;
  char type;
  uint32_t response_time;
  McuDof transform;
  McuDof forward_kinematic;
  float motors_target_velocity[API_MOTOR_COUNT];
  float motors_desire_velocity[API_MOTOR_COUNT];
  float motors_actual_velocity[API_MOTOR_COUNT];
  uint32_t motors_ccr[API_MOTOR_COUNT];
  McuPower battery1;
  McuPower battery2;
  bool software_emergency_stop_enabled;
  bool hardware_emergency_stop_enabled;
  bool bettery_low_emergency_stop_enabled;
  uint8_t header3;
  uint8_t header4;
} McuData;

typedef struct {
  uint8_t header1;
  uint8_t header2;
  char type;
  uint32_t serial_number1;
  uint32_t serial_number2;
  uint32_t serial_number3;
  uint8_t header3;
  uint8_t header4;
} McuSerialNumber;

typedef struct {
  uint8_t header1;
  uint8_t header2;
  char type;
  float pid_speed[PID_LEVEL];
  float p[PID_LEVEL][API_MOTOR_COUNT];
  float i[PID_LEVEL][API_MOTOR_COUNT];
  float d[PID_LEVEL][API_MOTOR_COUNT];
  float motors_maximum_speed[API_MOTOR_COUNT];
  uint8_t header3;
  uint8_t header4;
} McuPid;

/*
 * PC to MCU communication
 */

// Emergency stop
typedef struct {
  char command;
  bool enable;
} McuSoftwareEmergencyStopCommand;

// Motor control
typedef struct {
  char command;
  McuDof velocity;
} McuInverseKinematicsCommand;

typedef struct {
  char command;
  uint8_t motor;
  float velocity;
} McuMotorCommand;

// PID control
typedef struct {
  char command;
  float pid_speed[PID_LEVEL];
} McuSetPidSpeedCommand;

typedef struct {
  char command;
} McuGetPidCommand;

typedef struct {
  char command;
  uint8_t motor;
  uint8_t level;
  float p;
  float i;
  float d;
} McuSetPidCommand;

typedef struct {
  char command;
} McuSavePidCommand;

typedef struct {
  char command;
  float speed[API_MOTOR_COUNT];
} McuSetMotorMaximumSpeedCommand;

// Other
typedef struct {
  char command;
} McuGetSerialNumberCommand;

typedef struct {
  char command;
} McuResetTransformCommand;

#pragma pack(pop)

#endif