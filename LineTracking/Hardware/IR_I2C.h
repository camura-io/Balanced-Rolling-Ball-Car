#ifndef __IR_I2C_H
#define __IR_I2C_H

#include "stm32f10x.h"

/* 8 路红外模块的 7 位 I2C 地址 */
#define IR_I2C_ADDRESS_7BIT       0x12U

/* 模块寄存器 */
#define IR_REG_ADJUST_MODE        0x01U
#define IR_REG_DIGITAL_DATA       0x30U

/*
 * 使用 STM32F103 的 I2C2：
 * PB10 -> SCL
 * PB11 -> SDA
 */
void IR_I2C_Init(void);

/* 成功返回 1，超时或总线异常返回 0。 */
uint8_t IR_I2C_ReadRaw(uint8_t *raw_data);
uint8_t IR_I2C_SetAdjustMode(uint8_t enable);

#endif
