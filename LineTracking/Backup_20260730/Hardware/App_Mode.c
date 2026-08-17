#include "App_Mode.h"
#include "stm32f10x.h"
#include "pwm.h"
#include "encoder.h"
#include "OLED.h"
#include "Timer1.h"
#include "IR_I2C.h"
#include "LineTracking.h"

/* 运行与循迹控制宏定义 */
#define TRACK_BASE_SPEED_CMS      35.0f
#define TRACK_SLOW_SPEED_CMS      20.0f

/* === 模式 5 和模式 6 的专有速度与加速度控制参数 === */
#define MODE56_TARGET_SPEED_CMS   30.0f   /* 正常循迹巡航目标速度：30 cm/s */
#define MODE56_START_SPEED_CMS    10.0f   /* 起步加速初速度：10 cm/s（匹配最低有效轮速） */
#define MODE56_MIN_SPEED_CMS      10.0f   /* 减速刹车下限速度：10 cm/s，降至此值直接全停 */

/* 
 * 斜坡加减速步长（基于 10 ms 控制周期）：
 * - 0.10f cm/s 对应加速度 a = 0.10 / 0.01s = 10 cm/s^2 (需 2.0s 达到 30cm/s)
 * - 0.08f cm/s 对应减速度 a = 0.08 / 0.01s =  8 cm/s^2 (需 2.5s 从 30cm/s 降至 10cm/s)
 */
#define MODE56_ACCEL_STEP_CMS     0.04f   /* 10 ms 周期的加速步长 */
#define MODE56_DECEL_STEP_CMS     0.04f   /* 10 ms 周期的减速步长 */


#define TRACK_SLOW_DISTANCE_CM    580.0f
/* === 模式 4 参数配置 === */
#define MODE4_TEST_SPEED_CMS      30.0f   /* 模式 4 巡航目标速度：30 cm/s */
#define MODE4_TEST_DISTANCE_CM    150.0f  /* 模式 4 减速触发距离：150 cm */

#define MODE_MIN                  1U
#define MODE_MAX                  6U
#define BUTTON_DEBOUNCE_TICKS     3U     /* 3 * 10ms = 30ms */
#define RUNTIME_REFRESH_TICKS     10U    /* 10 * 10ms = 0.1s */

typedef struct
{
    uint8_t last_sample;
    uint8_t stable_state;
    uint8_t same_count;
} ButtonDebounce_t;

static uint8_t Button_GetPressedEvent(ButtonDebounce_t *button, uint8_t current_sample)
{
    if (current_sample == button->last_sample)
    {
        if (button->same_count < BUTTON_DEBOUNCE_TICKS)
        {
            button->same_count++;
        }
    }
    else
    {
        button->last_sample = current_sample;
        button->same_count = 1U;
    }

    if ((button->same_count >= BUTTON_DEBOUNCE_TICKS) &&
        (button->stable_state != current_sample))
    {
        button->stable_state = current_sample;

        if (current_sample != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint32_t Wait_Next_10ms(uint32_t previous_tick)
{
    uint32_t current_tick;

    do
    {
        current_tick = Control_Tick_10ms;
    }
    while (current_tick == previous_tick);

    return current_tick;
}

static void OLED_ShowModeName(uint8_t mode)
{
    switch (mode)
    {
        case 1U:
            OLED_ShowString(1, 1, "MODE1: SELECT   ");
            break;
        case 2U:
            OLED_ShowString(1, 1, "MODE2: IR TRACK ");
            break;
        case 3U:
            OLED_ShowString(1, 1, "MODE3: RESERVED ");
            break;
        case 4U:
            OLED_ShowString(1, 1, "MODE4: DIST TEST");
            break;
				case 5U:
            OLED_ShowString(1, 1, "MODE5: TRACK 25 ");
            break;
        case 6U:
            OLED_ShowString(1, 1, "MODE6: TRACK 25 ");
            break;
        default:
            OLED_ShowString(1, 1, "MODE?: ERROR    ");
            break;
    }
}

static void OLED_ShowModeSelection(uint8_t mode)
{
    OLED_ShowModeName(mode);
    OLED_ShowString(2, 1, "PB12: Next      ");
    OLED_ShowString(3, 1, "PB13: Confirm   ");

    if (mode == 1U)
    {
        OLED_ShowString(4, 1, "Choose MODE 2-6 ");
    }
    else
    {
        OLED_ShowString(4, 1, "PB13 to start   ");
    }
}

static void OLED_ShowRuntime(uint32_t start_tick)
{
    uint32_t elapsed_tenths;

    elapsed_tenths = (Control_Tick_10ms - start_tick) / 10U;
    OLED_ShowNum(2, 7, (elapsed_tenths / 10U) % 1000U, 3);
    OLED_ShowNum(2, 11, elapsed_tenths % 10U, 1);
}

static void OLED_ShowDistance(void)
{
    uint32_t distance_tenths;

    distance_tenths =
        (uint32_t)(Odometer_Get_Distance_CM() * 10.0f + 0.5f);

    if (distance_tenths > 9999U)
    {
        distance_tenths = 9999U;
    }

    OLED_ShowNum(3, 7, (distance_tenths / 10U) % 1000U, 3);
    OLED_ShowNum(3, 11, distance_tenths % 10U, 1);
}

static uint8_t Wait_IR_Module_Ready(uint8_t *raw_data)
{
    uint8_t attempt;
    uint32_t wait_tick = Control_Tick_10ms;

    for (attempt = 0U; attempt < 20U; attempt++)
    {
        if (IR_I2C_ReadRaw(raw_data))
        {
            return 1U;
        }

        while (Control_Tick_10ms == wait_tick)
        {
        }
        wait_tick = Control_Tick_10ms;
    }

    return 0U;
}



static void Run_Mode2_IR_Tracking(void)
{
    uint8_t ir_raw;
    uint32_t last_tracking_tick;
    uint32_t start_tick;
    uint32_t last_runtime_refresh_tick;
    uint8_t speed_reduced = 0U;

    start_tick = Control_Tick_10ms;

    OLED_ShowModeName(2U);
    OLED_ShowString(2, 1, "Time: 000.0s    ");
    OLED_ShowString(3, 1, "Dist: 000.0cm   ");
    OLED_ShowString(4, 1, "Speed: 00cm/s   ");
    OLED_ShowNum(4, 8, (uint32_t)TRACK_BASE_SPEED_CMS, 2);
    OLED_ShowRuntime(start_tick);
    OLED_ShowDistance();

    if (!Wait_IR_Module_Ready(&ir_raw))
    {
        Set_Target_Speed(0, 0);
        OLED_ShowRuntime(start_tick);
        OLED_ShowString(4, 1, "State: I2C Err  ");

        while (1)
        {
        }
    }

    Odometer_Reset();
    LineTracking_Init(TRACK_BASE_SPEED_CMS);
    LineTracking_Start();
    last_tracking_tick = Control_Tick_10ms;
    last_runtime_refresh_tick = last_tracking_tick;
    OLED_ShowRuntime(start_tick);
    OLED_ShowDistance();

    while (1)
    {
        uint32_t current_tick;

        current_tick = Wait_Next_10ms(last_tracking_tick);
        last_tracking_tick = current_tick;

        if ((speed_reduced == 0U) &&
            (Odometer_Get_Distance_CM() >= TRACK_SLOW_DISTANCE_CM))
        {
            speed_reduced = 1U;
            LineTracking_SetBaseSpeed(TRACK_SLOW_SPEED_CMS);
            OLED_ShowNum(4, 8, (uint32_t)TRACK_SLOW_SPEED_CMS, 2);
        }

        if (IR_I2C_ReadRaw(&ir_raw))
        {
            LineTracking_Update(ir_raw);
        }
        else
        {
            LineTracking_OnI2CFailure();
        }

        if ((current_tick - last_runtime_refresh_tick) >= RUNTIME_REFRESH_TICKS)
        {
            last_runtime_refresh_tick = current_tick;
            OLED_ShowRuntime(start_tick);
            OLED_ShowDistance();
        }

        if (LineTracking_GetState() == LINE_TRACK_LINE_LOST)
        {
            OLED_ShowRuntime(start_tick);
            OLED_ShowDistance();
            OLED_ShowString(4, 1, "State: LineLost ");
            while (1)
            {
            }
        }

        if (LineTracking_GetState() == LINE_TRACK_I2C_FAULT)
        {
            OLED_ShowRuntime(start_tick);
            OLED_ShowDistance();
            OLED_ShowString(4, 1, "State: I2C Err  ");
            while (1)
            {
            }
        }

        if (LineTracking_GetMarkerCount() >= 2U)
        {
            LineTracking_Stop();
            OLED_ShowRuntime(start_tick);
            OLED_ShowDistance();
            OLED_ShowString(4, 1, "State: Finished ");
            while (1)
            {
            }
        }
    }
}

static void Run_Mode4_Distance_Test(void)
{
    uint8_t ir_raw;
    uint32_t start_tick;
    uint32_t last_tracking_tick;
    uint32_t last_runtime_refresh_tick;
    
    uint8_t timer_stopped = 0U;                   /* 150cm 到达标志（同时兼作时间锁存标志） */
    float current_speed = MODE56_START_SPEED_CMS; /* 起步初速度：10 cm/s */

    start_tick = Control_Tick_10ms;

    /* 1. UI 初始化 */
    OLED_ShowModeName(4U);
    OLED_ShowString(2, 1, "Time: 000.0s    ");
    OLED_ShowString(3, 1, "Dist: 000.0cm   ");
    OLED_ShowString(4, 1, "Speed: 00cm/s   ");
    OLED_ShowNum(4, 8, (uint32_t)MODE56_START_SPEED_CMS, 2);
    OLED_ShowRuntime(start_tick);
    OLED_ShowDistance();

    /* 2. 检测并等待红外循迹模块响应 */
    if (!Wait_IR_Module_Ready(&ir_raw))
    {
        Set_Target_Speed(0, 0);
        OLED_ShowRuntime(start_tick);
        OLED_ShowString(4, 1, "State: I2C Err  ");
        while (1)
        {
        }
    }

    /* 3. 复位编码器并启动底层循迹算法 */
    Odometer_Reset();
    LineTracking_Init(MODE56_START_SPEED_CMS);
    LineTracking_Start();

    last_tracking_tick = Control_Tick_10ms;
    last_runtime_refresh_tick = last_tracking_tick;

    while (1)
    {
        uint32_t current_tick;

        current_tick = Wait_Next_10ms(last_tracking_tick);
        last_tracking_tick = current_tick;

        /* 4. 传感器数据采集与 PID/分段循迹输出解算 */
        if (IR_I2C_ReadRaw(&ir_raw))
        {
            LineTracking_Update(ir_raw);
        }
        else
        {
            LineTracking_OnI2CFailure();
        }

        /* 5. 监测里程计脉冲：行驶达到 150cm 时，立即锁定 OLED 时间显示 */
        if ((timer_stopped == 0U) && (Odometer_Get_Distance_CM() >= MODE4_TEST_DISTANCE_CM))
        {
            timer_stopped = 1U;
            OLED_ShowRuntime(start_tick); /* 记录并锁定到达 150cm 那一刻的时间 */
        }

        /* 6. 斜坡速度规划控制 */
        if (timer_stopped == 0U)
        {
            /* 匀速/加速段：斜坡提速至 30cm/s 目标巡航车速 */
            if (current_speed < MODE4_TEST_SPEED_CMS)
            {
                current_speed += MODE56_ACCEL_STEP_CMS;
                if (current_speed > MODE4_TEST_SPEED_CMS)
                {
                    current_speed = MODE4_TEST_SPEED_CMS;
                }
                LineTracking_SetBaseSpeed(current_speed);
                OLED_ShowNum(4, 8, (uint32_t)current_speed, 2);
            }
        }
        else
        {
            /* 刹车段：超过 150cm 后斜坡减速，降至 10cm/s 后完全停车 */
            current_speed -= MODE56_DECEL_STEP_CMS;

            if (current_speed <= MODE56_MIN_SPEED_CMS)
            {
                LineTracking_Stop();
                OLED_ShowDistance();
                OLED_ShowString(4, 1, "State: Finished ");
                while (1)
                {
                    /* 任务完成，停机挂起 */
                }
            }
            else
            {
                LineTracking_SetBaseSpeed(current_speed);
                OLED_ShowNum(4, 8, (uint32_t)current_speed, 2);
            }
        }

        /* 7. OLED 界面显示刷新（100ms 周期） */
        if ((current_tick - last_runtime_refresh_tick) >= RUNTIME_REFRESH_TICKS)
        {
            last_runtime_refresh_tick = current_tick;
            
            /* 仅在未达到 150cm 前更新时间，到达后冻结时间刷新 */
            if (timer_stopped == 0U)
            {
                OLED_ShowRuntime(start_tick);
            }
            OLED_ShowDistance();
        }

        /* 8. 脱线及硬件故障检测 */
        if (LineTracking_GetState() == LINE_TRACK_LINE_LOST)
        {
            if (timer_stopped == 0U)
            {
                OLED_ShowRuntime(start_tick);
            }
            OLED_ShowDistance();
            OLED_ShowString(4, 1, "State: LineLost ");
            while (1)
            {
            }
        }

        if (LineTracking_GetState() == LINE_TRACK_I2C_FAULT)
        {
            if (timer_stopped == 0U)
            {
                OLED_ShowRuntime(start_tick);
            }
            OLED_ShowDistance();
            OLED_ShowString(4, 1, "State: I2C Err  ");
            while (1)
            {
            }
        }
    }
}

static void Run_Mode5_6_IR_Tracking(uint8_t mode)
{
    uint8_t ir_raw;
    uint32_t last_tracking_tick;
    uint32_t start_tick;
    uint32_t last_runtime_refresh_tick;
    uint8_t timer_stopped = 0U;
    
    /* 初始运行速度设置为起步初速度 */
    float current_speed = MODE56_START_SPEED_CMS;

    start_tick = Control_Tick_10ms;

    OLED_ShowModeName(mode);
    OLED_ShowString(2, 1, "Time: 000.0s    ");
    OLED_ShowString(3, 1, "Dist: 000.0cm   ");
    OLED_ShowString(4, 1, "Speed: 00cm/s   ");
    OLED_ShowNum(4, 8, (uint32_t)MODE56_START_SPEED_CMS, 2);
    OLED_ShowRuntime(start_tick);
    OLED_ShowDistance();

    if (!Wait_IR_Module_Ready(&ir_raw))
    {
        Set_Target_Speed(0, 0);
        OLED_ShowRuntime(start_tick);
        OLED_ShowString(4, 1, "State: I2C Err  ");

        while (1)
        {
        }
    }

    Odometer_Reset();
    /* 使用初始初速度初始化底层循迹控制层 */
    LineTracking_Init(MODE56_START_SPEED_CMS);
    LineTracking_Start();
    last_tracking_tick = Control_Tick_10ms;
    last_runtime_refresh_tick = last_tracking_tick;

    while (1)
    {
        uint32_t current_tick;

        current_tick = Wait_Next_10ms(last_tracking_tick);
        last_tracking_tick = current_tick;

        /* 1. 读取传感器并更新循迹解算 */
        if (IR_I2C_ReadRaw(&ir_raw))
        {
            LineTracking_Update(ir_raw);
        }
        else
        {
            LineTracking_OnI2CFailure();
        }

        /* 2. 检测到达停止线（累计检测到 2 次横向标线即认定一圈结束） */
        if ((timer_stopped == 0U) && (LineTracking_GetMarkerCount() >= 2U))
        {
            timer_stopped = 1U;
            OLED_ShowRuntime(start_tick);
        }

        /* 3. 分段斜坡速度规划：运行加速期 vs 撞线减速期 */
        if (timer_stopped == 0U)
        {
            /* 阶段一：斜坡起步加速，升至目标巡航车速时封顶 */
            if (current_speed < MODE56_TARGET_SPEED_CMS)
            {
                current_speed += MODE56_ACCEL_STEP_CMS;
                if (current_speed > MODE56_TARGET_SPEED_CMS)
                {
                    current_speed = MODE56_TARGET_SPEED_CMS;
                }
                LineTracking_SetBaseSpeed(current_speed);
                OLED_ShowNum(4, 8, (uint32_t)current_speed, 2);
            }
        }
        else
        {
            /* 阶段二：撞线后的斜坡缓冲减速控制 */
            current_speed -= MODE56_DECEL_STEP_CMS;

            if (current_speed <= MODE56_MIN_SPEED_CMS)
            {
                LineTracking_Stop();
                OLED_ShowDistance();
                OLED_ShowString(4, 1, "State: Finished ");
                while (1)
                {
                }
            }
            else
            {
                LineTracking_SetBaseSpeed(current_speed);
                OLED_ShowNum(4, 8, (uint32_t)current_speed, 2);
            }
        }

        /* 4. OLED 周期性刷新控制 */
        if ((current_tick - last_runtime_refresh_tick) >= RUNTIME_REFRESH_TICKS)
        {
            last_runtime_refresh_tick = current_tick;
            if (timer_stopped == 0U)
            {
                OLED_ShowRuntime(start_tick);
            }
            OLED_ShowDistance();
        }

        /* 5. 异常脱线与硬件故障检测 */
        if (LineTracking_GetState() == LINE_TRACK_LINE_LOST)
        {
            if (timer_stopped == 0U)
            {
                OLED_ShowRuntime(start_tick);
            }
            OLED_ShowDistance();
            OLED_ShowString(4, 1, "State: LineLost ");
            while (1)
            {
            }
        }

        if (LineTracking_GetState() == LINE_TRACK_I2C_FAULT)
        {
            if (timer_stopped == 0U)
            {
                OLED_ShowRuntime(start_tick);
            }
            OLED_ShowDistance();
            OLED_ShowString(4, 1, "State: I2C Err  ");
            while (1)
            {
            }
        }
    }
}


static void Run_Reserved_Mode(uint8_t mode)
{
    uint32_t start_tick;
    uint32_t last_tick;
    uint32_t last_runtime_refresh_tick;

    start_tick = Control_Tick_10ms;
    Set_Target_Speed(0, 0);

    OLED_ShowModeName(mode);
    OLED_ShowString(2, 1, "Time: 000.0s    ");
    OLED_ShowString(3, 1, "Function: Empty ");
    OLED_ShowString(4, 1, "State: Running  ");

    last_tick = Control_Tick_10ms;
    last_runtime_refresh_tick = last_tick;
    OLED_ShowRuntime(start_tick);

    while (1)
    {
        last_tick = Wait_Next_10ms(last_tick);

        if ((last_tick - last_runtime_refresh_tick) >= RUNTIME_REFRESH_TICKS)
        {
            last_runtime_refresh_tick = last_tick;
            OLED_ShowRuntime(start_tick);
        }
    }
}

/* ================== 外露业务接口 ================== */

void App_Mode_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint8_t App_Mode_Select(void)
{
    uint8_t selected_mode = MODE_MIN;
    uint8_t next_pressed;
    uint8_t confirm_pressed;
    uint32_t last_tick = Control_Tick_10ms;
    ButtonDebounce_t next_button = {0U, 0U, 0U};
    ButtonDebounce_t confirm_button = {0U, 0U, 0U};

    OLED_ShowModeSelection(selected_mode);

    while (1)
    {
        last_tick = Wait_Next_10ms(last_tick);

        next_pressed = Button_GetPressedEvent(
            &next_button,
            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12));

        confirm_pressed = Button_GetPressedEvent(
            &confirm_button,
            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13));

        if (confirm_pressed != 0U)
        {
            if (selected_mode != 1U)
            {
                return selected_mode;
            }
        }
        else if (next_pressed != 0U)
        {
            selected_mode++;
            if (selected_mode > MODE_MAX)
            {
                selected_mode = MODE_MIN;
            }

            OLED_ShowModeSelection(selected_mode);
        }
    }
}

void App_Mode_Run(uint8_t mode)
{
    switch (mode)
    {
        case 2U:
            Run_Mode2_IR_Tracking();
            break;

        case 4U:
            Run_Mode4_Distance_Test();
            break;

        /* === 模式 5 和模式 6 统一指向带斜坡减速的循迹实现 === */
        case 5U:
        case 6U:
            Run_Mode5_6_IR_Tracking(mode);
            break;

        case 3U:
            Run_Reserved_Mode(mode);
            break;

        default:
            Set_Target_Speed(0, 0);
            OLED_ShowString(1, 1, "MODE: ERROR     ");
            OLED_ShowString(2, 1, "Time: --        ");
            OLED_ShowString(3, 1, "Check selection ");
            OLED_ShowString(4, 1, "State: Stopped  ");
            while (1)
            {
            }
    }
}
