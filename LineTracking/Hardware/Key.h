#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

#define KEY_NONE           0U
#define KEY_MODE_SWITCH    1U
#define KEY_MODE_CONFIRM   2U

void Key_Init(void);
uint8_t Key_GetNum(void);

#endif
