#ifndef APP_K230_H
#define APP_K230_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_K230_EVENT_NONE      0x00U
#define APP_K230_EVENT_TARGET    0x01U
#define APP_K230_EVENT_NO_TARGET 0x02U

typedef struct
{
  int16_t cx;
  int16_t cy;
  int16_t dx;
  int16_t dy;
  int16_t w;
  int16_t h;
  int16_t pos_mm;
} AppK230_Target_t;

void AppK230_Init(UART_HandleTypeDef *huart);
void AppK230_StartRx(void);
uint8_t AppK230_ProcessLine(void);
void AppK230_RxCpltCallback(UART_HandleTypeDef *huart);
void AppK230_ErrorCallback(UART_HandleTypeDef *huart);
void AppK230_Invalidate(void);

uint8_t AppK230_IsTargetValid(void);
uint8_t AppK230_HasFreshTarget(uint32_t timeout_ms);
uint32_t AppK230_GetAgeMs(void);
uint32_t AppK230_GetFrames(void);
uint32_t AppK230_GetLostFrames(void);
int16_t AppK230_GetPosMm(void);
AppK230_Target_t AppK230_GetTarget(void);

#ifdef __cplusplus
}
#endif

#endif
