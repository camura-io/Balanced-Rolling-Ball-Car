#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include <stdint.h>

HAL_StatusTypeDef OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearLine(uint8_t line);
void OLED_ShowChar(uint8_t line, uint8_t column, char ch);
void OLED_ShowString(uint8_t line, uint8_t column, const char *string);
void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length);
void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
uint8_t OLED_IsOk(void);

#endif
