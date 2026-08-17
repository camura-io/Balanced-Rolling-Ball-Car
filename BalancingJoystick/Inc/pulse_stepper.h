#ifndef __PULSE_STEPPER_H
#define __PULSE_STEPPER_H

#include "main.h"

HAL_StatusTypeDef PulseStepper_Init(void);
void PulseStepper_Process1ms(void);
void PulseStepper_StopX(void);
void PulseStepper_StopY(void);
void PulseStepper_StopAll(void);
HAL_StatusTypeDef PulseStepper_RunXSpeed(uint8_t dir, uint16_t speed_rpm);
HAL_StatusTypeDef PulseStepper_RunYSpeed(uint8_t dir, uint16_t speed_rpm);

#endif
