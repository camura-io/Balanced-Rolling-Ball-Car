#include "Timer1.h"
#include "encoder.h"
#include "moto.h"
#include "pwm.h"

/*
 * 车轮直径 67mm、减速比 30、11线AB编码器、四倍频：
 * 输出轴每圈计数 = 30 * 11 * 4 = 1320
 * 车轮标称周长   = PI * 6.7cm
 *
 * ODOMETER_CALIBRATION 用于修正轮胎压缩、打滑和减速比误差。
 * 首次1.5m测试保持1.0000，实测后再按结果修改。
 */
#define PI_F                         3.1415926f
#define WHEEL_DIAMETER_CM            6.7f
#define GEAR_RATIO                   30.0f
#define ENCODER_LINES                11.0f
#define ENCODER_QUADRATURE           4.0f
#define ODOMETER_CALIBRATION         1.0000f

#define PULSE_PER_WHEEL_REV    \
    (GEAR_RATIO * ENCODER_LINES * ENCODER_QUADRATURE)
#define PULSE_PER_CM           \
    (PULSE_PER_WHEEL_REV /     \
    (PI_F * WHEEL_DIAMETER_CM * ODOMETER_CALIBRATION))
#define VEL_CMS_TO_PULSE       (PULSE_PER_CM * 0.01f)
#define PULSE_TO_VEL_CMS       (100.0f / PULSE_PER_CM)

/*
 * 达到目标前提前多少脉冲开始制动。
 * 先保持为 0；完成实车测试后，如果仍有固定制动超程再进行标定。
 */
#define DISTANCE_BRAKE_LEAD_PULSE    0U

/* 连续 100ms 无编码器脉冲后，认为车辆已经停稳。 */
#define STOP_STABLE_TICKS            10U

volatile int Target_Speed_A = 0;
volatile int Target_Speed_B = 0;
volatile int Encoder_A_Val  = 0;
volatile int Encoder_B_Val  = 0;
volatile int Motor_PWM_A    = 0;
volatile int Motor_PWM_B    = 0;
volatile uint32_t Control_Tick_10ms = 0;

static volatile uint8_t Motion_State = MOTION_IDLE;
static volatile uint8_t Motion_Done = 1U;
static volatile uint8_t Stop_Stable_Count = 0U;

static volatile uint32_t Motion_Target_Ticks = 0U;
static volatile uint32_t Motion_Target_Pulse = 0U;
static volatile uint32_t Motion_Elapsed_Ticks = 0U;

static volatile int32_t Motion_Total_Pulse_A = 0;
static volatile int32_t Motion_Total_Pulse_B = 0;

/*
 * 独立总里程计数：每个10ms采样周期累加左右轮脉冲绝对值。
 * 它不依赖 Motion_State，因此循迹和以后新增的所有模式均可使用。
 */
static volatile uint32_t Odometer_Total_Pulse_A = 0U;
static volatile uint32_t Odometer_Total_Pulse_B = 0U;

static int Speed_CMS_To_Pulse(float speed_cms)
{
    return (int)(speed_cms * VEL_CMS_TO_PULSE
               + (speed_cms >= 0.0f ? 0.5f : -0.5f));
}

static uint32_t Abs_I32(int32_t value)
{
    return (value >= 0) ? (uint32_t)value : (uint32_t)(-value);
}

static uint32_t Enter_Critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Exit_Critical(uint32_t primask)
{
    if ((primask & 0x01U) == 0U)
    {
        __enable_irq();
    }
}

static void Reset_Motion_Data(void)
{
    Motion_Total_Pulse_A = 0;
    Motion_Total_Pulse_B = 0;
    Motion_Elapsed_Ticks = 0U;
    Stop_Stable_Count = 0U;
    Motion_Done = 0U;
}

void Set_Target_Speed(int speed_a, int speed_b)
{
    uint32_t primask = Enter_Critical();

    Target_Speed_A = speed_a;
    Target_Speed_B = speed_b;

    Exit_Critical(primask);
}

void Set_Target_Speed_CMS(float speed_a_cms, float speed_b_cms)
{
    Set_Target_Speed(Speed_CMS_To_Pulse(speed_a_cms),
                     Speed_CMS_To_Pulse(speed_b_cms));
}

float Get_Actual_Speed_A_CMS(void)
{
    return (float)Encoder_A_Val * PULSE_TO_VEL_CMS;
}

float Get_Actual_Speed_B_CMS(void)
{
    return (float)Encoder_B_Val * PULSE_TO_VEL_CMS;
}

void Odometer_Reset(void)
{
    uint32_t primask = Enter_Critical();

    Odometer_Total_Pulse_A = 0U;
    Odometer_Total_Pulse_B = 0U;

    Exit_Critical(primask);
}

uint32_t Odometer_Get_Total_Pulse_A(void)
{
    return Odometer_Total_Pulse_A;
}

uint32_t Odometer_Get_Total_Pulse_B(void)
{
    return Odometer_Total_Pulse_B;
}

uint32_t Odometer_Get_Center_Pulse(void)
{
    uint32_t pulse_a;
    uint32_t pulse_b;
    uint32_t primask = Enter_Critical();

    pulse_a = Odometer_Total_Pulse_A;
    pulse_b = Odometer_Total_Pulse_B;

    Exit_Critical(primask);

    return (pulse_a + pulse_b) / 2U;
}

float Odometer_Get_Distance_CM(void)
{
    return (float)Odometer_Get_Center_Pulse() / PULSE_PER_CM;
}

void Motion_Start_Distance_CMS(float distance_cm, float speed_cms)
{
    float distance_abs;
    float speed_abs;
    int direction;
    int target_speed;
    uint32_t target_pulse;
    uint32_t primask;

    direction = (distance_cm < 0.0f) ? -1 : 1;
    distance_abs = (distance_cm < 0.0f) ? -distance_cm : distance_cm;
    speed_abs = (speed_cms < 0.0f) ? -speed_cms : speed_cms;

    target_pulse = (uint32_t)(distance_abs * PULSE_PER_CM + 0.5f);
    target_speed = Speed_CMS_To_Pulse(speed_abs) * direction;

    primask = Enter_Critical();

    Reset_Motion_Data();
    Motion_Target_Pulse = target_pulse;
    Motion_Target_Ticks = 0U;

    if ((target_pulse == 0U) || (target_speed == 0))
    {
        Target_Speed_A = 0;
        Target_Speed_B = 0;
        Motion_State = MOTION_IDLE;
        Motion_Done = 1U;
    }
    else
    {
        Target_Speed_A = target_speed;
        Target_Speed_B = target_speed;
        Motion_State = MOTION_RUN_DISTANCE;
    }

    Exit_Critical(primask);
}

void Motion_Start_Time_CMS(float speed_a_cms,
                           float speed_b_cms,
                           uint32_t duration_ms)
{
    uint32_t target_ticks = (duration_ms + 9U) / 10U;
    int target_a = Speed_CMS_To_Pulse(speed_a_cms);
    int target_b = Speed_CMS_To_Pulse(speed_b_cms);
    uint32_t primask = Enter_Critical();

    Reset_Motion_Data();
    Motion_Target_Ticks = target_ticks;
    Motion_Target_Pulse = 0U;

    if ((target_ticks == 0U) || ((target_a == 0) && (target_b == 0)))
    {
        Target_Speed_A = 0;
        Target_Speed_B = 0;
        Motion_State = MOTION_IDLE;
        Motion_Done = 1U;
    }
    else
    {
        Target_Speed_A = target_a;
        Target_Speed_B = target_b;
        Motion_State = MOTION_RUN_TIME;
    }

    Exit_Critical(primask);
}

void Motion_Stop(void)
{
    uint32_t primask = Enter_Critical();

    Target_Speed_A = 0;
    Target_Speed_B = 0;

    if ((Motion_State == MOTION_RUN_DISTANCE)
     || (Motion_State == MOTION_RUN_TIME))
    {
        Motion_State = MOTION_BRAKING;
        Stop_Stable_Count = 0U;
    }
    else if (Motion_State == MOTION_IDLE)
    {
        Motion_Done = 1U;
    }

    Exit_Critical(primask);
}

uint8_t Motion_Get_State(void)
{
    return Motion_State;
}

uint8_t Motion_Is_Done(void)
{
    return Motion_Done;
}

uint32_t Motion_Get_Elapsed_Ticks(void)
{
    return Motion_Elapsed_Ticks;
}

int32_t Motion_Get_Total_Pulse_A(void)
{
    return Motion_Total_Pulse_A;
}

int32_t Motion_Get_Total_Pulse_B(void)
{
    return Motion_Total_Pulse_B;
}

uint32_t Motion_Get_Center_Pulse(void)
{
    int32_t pulse_a;
    int32_t pulse_b;
    uint32_t primask = Enter_Critical();

    pulse_a = Motion_Total_Pulse_A;
    pulse_b = Motion_Total_Pulse_B;

    Exit_Critical(primask);

    return (Abs_I32(pulse_a) + Abs_I32(pulse_b)) / 2U;
}

float Motion_Get_Distance_CMS(void)
{
    return (float)Motion_Get_Center_Pulse() / PULSE_PER_CM;
}

void Timer1_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_InternalClockConfig(TIM1);
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM1, ENABLE);
}

void TIM1_UP_IRQHandler(void)
{
    uint32_t center_pulse;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        Control_Tick_10ms++;

        Encoder_A_Val = Read_Encoder(2);
        Encoder_B_Val = Read_Encoder(4);

        Odometer_Total_Pulse_A += Abs_I32((int32_t)Encoder_A_Val);
        Odometer_Total_Pulse_B += Abs_I32((int32_t)Encoder_B_Val);

        if (Motion_State != MOTION_IDLE)
        {
            Motion_Total_Pulse_A += Encoder_A_Val;
            Motion_Total_Pulse_B += Encoder_B_Val;
        }

        if ((Motion_State == MOTION_RUN_DISTANCE)
         || (Motion_State == MOTION_RUN_TIME))
        {
            Motion_Elapsed_Ticks++;

            if (Motion_State == MOTION_RUN_DISTANCE)
            {
                center_pulse =
                    (Abs_I32(Motion_Total_Pulse_A)
                   + Abs_I32(Motion_Total_Pulse_B)) / 2U;

                if ((center_pulse + DISTANCE_BRAKE_LEAD_PULSE)
                    >= Motion_Target_Pulse)
                {
                    Target_Speed_A = 0;
                    Target_Speed_B = 0;
                    Motion_State = MOTION_BRAKING;
                    Stop_Stable_Count = 0U;
                }
            }
            else if (Motion_Elapsed_Ticks >= Motion_Target_Ticks)
            {
                Target_Speed_A = 0;
                Target_Speed_B = 0;
                Motion_State = MOTION_BRAKING;
                Stop_Stable_Count = 0U;
            }
        }
        else if (Motion_State == MOTION_BRAKING)
        {
            if ((Encoder_A_Val == 0) && (Encoder_B_Val == 0))
            {
                if (Stop_Stable_Count < STOP_STABLE_TICKS)
                {
                    Stop_Stable_Count++;
                }
            }
            else
            {
                Stop_Stable_Count = 0U;
            }

            if (Stop_Stable_Count >= STOP_STABLE_TICKS)
            {
                Motion_State = MOTION_IDLE;
                Motion_Done = 1U;
            }
        }

        Motor_PWM_A = Velocity_A(Target_Speed_A, Encoder_A_Val);
        Motor_PWM_B = Velocity_B(Target_Speed_B, Encoder_B_Val);
        Set_PWM(Motor_PWM_A, Motor_PWM_B);
    }
}
