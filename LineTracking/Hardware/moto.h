#ifndef __MOTO_H
#define __MOTO_H

#include "stm32f10x.h"

void moto(int mode);
int Velocity_A(int Target_Vel, int Current_Vel);
int Velocity_B(int Target_Vel, int Current_Vel);

#endif
