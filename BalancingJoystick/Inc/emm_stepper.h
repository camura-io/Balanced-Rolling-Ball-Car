#ifndef __EMM_STEPPER_H
#define __EMM_STEPPER_H

#include "main.h"

HAL_StatusTypeDef EMM_Stepper_Init(UART_HandleTypeDef *y_uart, UART_HandleTypeDef *x_uart);
void EMM_Stepper_Process1ms(void);
uint8_t EMM_Stepper_IsBusy(void);
HAL_StatusTypeDef EMM_Stepper_EnableY(uint8_t enable);
HAL_StatusTypeDef EMM_Stepper_EnableX(uint8_t enable);
HAL_StatusTypeDef EMM_Stepper_RunYSpeed(uint8_t dir, uint16_t speed_rpm, uint8_t acc);
HAL_StatusTypeDef EMM_Stepper_RunXSpeed(uint8_t dir, uint16_t speed_rpm, uint8_t acc);
HAL_StatusTypeDef EMM_Stepper_StopY(void);
HAL_StatusTypeDef EMM_Stepper_StopX(void);

#endif
