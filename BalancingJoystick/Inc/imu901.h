#ifndef __IMU901_H
#define __IMU901_H

#include "main.h"

typedef struct
{
  float roll;
  float pitch;
  float yaw;
  int16_t acc_x_mg;
  int16_t acc_y_mg;
  int16_t acc_z_mg;
  uint32_t angle_frames;
  uint32_t accel_frames;
  uint32_t valid_frames;
  uint32_t checksum_errors;
  uint32_t format_errors;
  uint32_t dropped_bytes;
  uint32_t last_angle_tick;
  uint32_t last_accel_tick;
} IMU901_Data_t;

HAL_StatusTypeDef IMU901_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef IMU901_EnsureAccelRange2G(void);
HAL_StatusTypeDef IMU901_SetAngleOutput100Hz(void);
void IMU901_Process(void);
void IMU901_GetData(IMU901_Data_t *data);
uint8_t IMU901_HasFreshAngle(uint32_t timeout_ms);
uint8_t IMU901_HasFreshAccel(uint32_t timeout_ms);
void IMU901_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void IMU901_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
