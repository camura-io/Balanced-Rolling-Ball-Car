#ifndef __VISION_TRACKER_H
#define __VISION_TRACKER_H

#include "main.h"

HAL_StatusTypeDef VisionTracker_Init(UART_HandleTypeDef *huart);
void VisionTracker_Process1ms(void);
uint8_t VisionTracker_IsActive(void);
uint8_t VisionTracker_HasFreshTarget(void);
void VisionTracker_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void VisionTracker_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
