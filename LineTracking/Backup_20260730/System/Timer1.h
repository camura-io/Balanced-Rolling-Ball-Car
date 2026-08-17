#ifndef __TIMER1_H
#define __TIMER1_H

#include "stm32f10x.h"

/* 运动状态 */
#define MOTION_IDLE              0U
#define MOTION_RUN_DISTANCE      1U
#define MOTION_RUN_TIME          2U
#define MOTION_BRAKING           3U

/*
 * 下列变量同时在主程序和 TIM1 中断中使用，必须声明为 volatile。
 * A 为左轮，B 为右轮。
 */
extern volatile int Target_Speed_A;
extern volatile int Target_Speed_B;
extern volatile int Encoder_A_Val;
extern volatile int Encoder_B_Val;
extern volatile int Motor_PWM_A;
extern volatile int Motor_PWM_B;
extern volatile uint32_t Control_Tick_10ms;

void Timer1_Init(u16 arr, u16 psc);

void Set_Target_Speed(int speed_a, int speed_b);
void Set_Target_Speed_CMS(float speed_a_cms, float speed_b_cms);

float Get_Actual_Speed_A_CMS(void);
float Get_Actual_Speed_B_CMS(void);

/*
 * 独立里程计。TIM1每10ms自动累计左右轮编码器脉冲绝对值，
 * 与 Motion_State 无关，所有运行模式均可复用。
 */
void Odometer_Reset(void);
uint32_t Odometer_Get_Total_Pulse_A(void);
uint32_t Odometer_Get_Total_Pulse_B(void);
uint32_t Odometer_Get_Center_Pulse(void);
float Odometer_Get_Distance_CM(void);

/*
 * 按距离运行：distance_cm 为正时前进，为负时后退。
 * speed_cms 只取大小，方向由 distance_cm 决定。
 */
void Motion_Start_Distance_CMS(float distance_cm, float speed_cms);

/* 按硬件时间运行，duration_ms 会向上取整到 10ms。 */
void Motion_Start_Time_CMS(float speed_a_cms,
                           float speed_b_cms,
                           uint32_t duration_ms);

/* 提前停止当前运动。 */
void Motion_Stop(void);

uint8_t Motion_Get_State(void);
uint8_t Motion_Is_Done(void);
uint32_t Motion_Get_Elapsed_Ticks(void);
int32_t Motion_Get_Total_Pulse_A(void);
int32_t Motion_Get_Total_Pulse_B(void);
uint32_t Motion_Get_Center_Pulse(void);
float Motion_Get_Distance_CMS(void);

#endif
						   