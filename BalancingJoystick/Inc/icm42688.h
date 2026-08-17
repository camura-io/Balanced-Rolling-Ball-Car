#ifndef __ICM42688_H
#define __ICM42688_H

#include "main.h"

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  uint8_t who_am_i;
  uint8_t ok;
  uint8_t active_mode;
  uint8_t probe_id[4];
  uint8_t shifted_id[4];
  uint8_t spi_status;
  uint8_t read_style;
  uint32_t frames;
  uint32_t errors;
  uint32_t int_count;
  uint32_t last_tick;
} ICM42688_Data_t;

HAL_StatusTypeDef ICM42688_Init(void);
void ICM42688_Process(void);
void ICM42688_GetData(ICM42688_Data_t *data);
uint8_t ICM42688_IsOk(void);
void ICM42688_EXTI_Callback(uint16_t gpio_pin);

#endif
