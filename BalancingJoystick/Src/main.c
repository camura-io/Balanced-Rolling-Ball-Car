/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu901.h"
#include "icm42688.h"
#include "OLED.h"
#include "app_k230.h"
#include "ball_balance_profile.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_TOGGLE_TICKS 1000U /* TIM6为1ms节拍，计满1000次就是1秒 */
#define LED_TRACK_TOGGLE_TICKS 200U /* K230跟踪开启但未锁定目标时快闪 */
#define APP_ICM_OLED_TEST_ONLY     1U /* 保留旧ICM测试开关；当前中心稳定改用MS901M。 */
#define APP_K230_MONITOR_ONLY      1U /* 打开USART2接收K230视觉数据，K2后才允许闭环控制。 */
#define APP_MS901M_ENABLE          1U /* MS901M接USART1，K1后采零点并参与中心稳定。 */
#define APP_ICM42688_ENABLE        0U /* 当前不用42688，避免未接线时阻塞/误报。 */
#define APP_MPU6050_ENABLE         0U /* 当前不用MPU6050。 */
#define APP_ACCEL_FEEDFORWARD_ENABLE 1U /* 中心稳定时叠加MS901M轴向加速度前馈。 */

#define MOTOR_TIMER_CLOCK_HZ       1000000UL /* TIM3: 240MHz/(239+1)=1MHz，480MHz主频下步进时基不变。 */
#define MOTOR_FULL_STEPS           200UL
#define MOTOR_MICROSTEPS           16UL
#define MOTOR_STEPS_PER_REV        (MOTOR_FULL_STEPS * MOTOR_MICROSTEPS)
#define MOTOR_MIN_RPM              1U
#define MOTOR_MAX_RPM              90U
#define BALL_TARGET_DEFAULT_MM     0L /* 启动后默认先稳在水管中心点，K230标定中心为0mm。 */
#define BALL_TARGET_MIN_MM         -120L /* K1/K3手调稳定位置允许到-12cm。 */
#define BALL_TARGET_MAX_MM         120L  /* K1/K3手调稳定位置允许到+12cm。 */
#define BALL_TARGET_KEY_STEP_MM    10L /* K5/K6每次把稳定目标加/减1cm。 */
#define BALL_TASK_POS_TARGET_MM    55L /* 赛题3：先从O运行到+5cm。 */
#define BALL_TASK_NEG_TARGET_MM    -45L /* 赛题3：折返到-5cm并稳定。 */
#define BALL_TASK_ARRIVE_ZONE_MM   5L /* 第三问到位收紧，不能在-4cm/+4cm边界就算完成。 */
#define BALL_TASK_ARRIVE_SPEED_MM  2L
#define BALL_TASK_ARRIVE_HOLD_MS   120U
#define BALL_TASK_TOTAL_LIMIT_MS   4900U /* 赛题3到4.5秒必须回平，之后不再做球位置调节。 */
#define BALL_OK_ZONE_MM            8L /* 控制目标留余量：小于0.8cm才允许低速保持，避免贴着1cm边界。 */
#define BALL_SLOW_ZONE_MM          10L /* 赛题1cm作为减速区：8~10mm还要动，但限制倾角。 */
#define BALL_DEADBAND_MM           2L /* 当前目标附近死区，避免识别抖动导致电机来回细碎动作。 */
#define BALL_PID_KP_NUM            10L /* 外环PID基础量，后面按启动/滑行/刹车三段再限幅。 */
#define BALL_PID_KI_NUM            1L
#define BALL_PID_KD_NUM            12L
#define BALL_PID_DEN               10L
#define BALL_PID_INTEGRAL_LIMIT    80L
#define BALL_CONTROL_SIGN          -1L /* 坐标约定：球要向负方向运动时，控制量必须为正角度。 */
#define BALL_LEVEL_ENABLE          1U /* MS901M加速度前馈中叠加K230球位置外环，负责中心保持。 */
#define BALL_LEVEL_OK_ZONE_MM      1L /* 定点保持：真正贴近0mm且几乎静止时回平。 */
#define BALL_LEVEL_RELEASE_MM      2L /* 超过0.2cm才允许常规小修，近中心另走阻尼区。 */
#define BALL_LEVEL_DAMP_ZONE_MM    4L /* 3~4mm附近不追视觉噪声，只给小保持/小反偏。 */
#define BALL_LEVEL_DAMP_BIAS_STEPS 4L /* 近中心往外跑时的小拉回角。 */
#define BALL_LEVEL_DAMP_BRAKE_STEPS 5L /* 近中心往中心跑时的小反向刹车角，防止过中心。 */
#define BALL_LEVEL_GUARD_MM        7L /* 超过0.7cm进入保护区，优先压回1cm以内。 */
#define BALL_LEVEL_LIMIT_MM        9L /* 预测接近0.9cm立即反向刹车，避免越过1cm。 */
#define BALL_LEVEL_HOLD_SPEED_MM   1L
#define BALL_LEVEL_PID_KP_NUM      8L
#define BALL_LEVEL_PID_KI_NUM      0L
#define BALL_LEVEL_PID_KD_NUM      26L
#define BALL_LEVEL_PID_DEN         10L
#define BALL_LEVEL_PID_I_LIMIT     30L
#define BALL_LEVEL_PREDICT_CYCLES  8L /* 约160ms预测，目标是提前守住±1cm，不等出界再追。 */
#define BALL_LEVEL_MAX_STEPS       30L /* 定点保持以稳为主，外环只给小角度偏置。 */
#define BALL_LEVEL_NEAR_MAX_STEPS  10L
#define BALL_LEVEL_GUARD_MIN_STEPS 14L
#define BALL_LEVEL_STATIC_MIN_STEPS 6L /* 偏离中心且低速卡住时给很小的持续修正。 */
#define BALL_LEVEL_BRAKE_ZONE_MM   18L
#define BALL_LEVEL_BRAKE_MIN_STEPS 16L
#define BALL_LEVEL_BRAKE_GAIN      5L
#define BALL_LEVEL_BRAKE_MAX_STEPS 34L
#define BALL_LEVEL_SLEW_STEPS      4L
#define BALL_LEVEL_BRAKE_SLEW_STEPS 12L
#define K3_CASCADE_Q8               256L /* K3位置/速度使用Q8定点，保留K230整毫米输入之间的细小变化。 */
#define K3_CASCADE_POS_DEADBAND_Q8  (1L * K3_CASCADE_Q8) /* 中心±1mm不再由位置环持续推球。 */
#define K3_CASCADE_HOLD_SPEED_Q8    (K3_CASCADE_Q8 / 8L) /* 静止判断约0.125mm/20ms。 */
#define K3_CASCADE_POS_KP_NUM       6L /* 外环：位置误差生成期望球速度。 */
#define K3_CASCADE_POS_KP_DEN       100L
#define K3_CASCADE_POS_KI_NUM       1L
#define K3_CASCADE_POS_KI_DEN       2400L
#define K3_CASCADE_POS_I_LIMIT_Q8   (160L * K3_CASCADE_Q8)
#define K3_CASCADE_VREF_MAX_Q8      (6L * K3_CASCADE_Q8 / 5L) /* 期望球速上限1.2mm/20ms。 */
#define K3_CASCADE_CENTER_BRAKE_START_MM 35L /* 两侧滚向中心时，从此距离开始连续收紧球速给定。 */
#define K3_CASCADE_CENTER_BRAKE_START_Q8 \
  (K3_CASCADE_CENTER_BRAKE_START_MM * K3_CASCADE_Q8)
#define K3_CASCADE_VEL_KP_NUM       24L /* 内环：球速度误差生成管子目标角。 */
#define K3_CASCADE_VEL_KP_DEN       100L
#define K3_CASCADE_VEL_KI_NUM       1L
#define K3_CASCADE_VEL_KI_DEN       400L
#define K3_CASCADE_VEL_KD_NUM       4L
#define K3_CASCADE_VEL_KD_DEN       100L
#define K3_CASCADE_VEL_I_LIMIT_Q8   (12L * K3_CASCADE_Q8)
#define K3_CASCADE_NEAR_POS_MAX_STEPS 30L /* 目标步数为正时，距中心4mm内的上限。 */
#define K3_CASCADE_NEAR_NEG_MAX_STEPS 32L /* 电机负方向需要更大角度，保留约20%裕量。 */
#define K3_CASCADE_POS_MAX_STEPS    60L /* 目标步数为正且超过4mm时的上限。 */
#define K3_CASCADE_NEG_MAX_STEPS    64L /* 目标步数为负且超过4mm时的上限。 */
#define K3_CASCADE_NEAR_POS_BRAKE_MAX_STEPS 46L /* 球减速时允许比正常推动更大的反向角。 */
#define K3_CASCADE_NEAR_NEG_BRAKE_MAX_STEPS 42L
#define K3_CASCADE_POS_BRAKE_MAX_STEPS 84L
#define K3_CASCADE_NEG_BRAKE_MAX_STEPS 84L
#define K3_CASCADE_SLEW_STEPS       6L
#define K3_CASCADE_BRAKE_SLEW_STEPS 18L
#define BALL_FILTER_NUM            3L /* 位置一阶滤波：3/4旧值+1/4新值，压住K230抖动。 */
#define BALL_FILTER_DEN            4L
#define BALL_PREDICT_CYCLES        14L /* 用约280ms预测位置，长距离提前刹车，避免冲过目标才反向。 */
#define BALL_HOLD_ZONE_MM          BALL_OK_ZONE_MM
#define BALL_HOLD_SPEED_MM         1L
#define BALL_BRAKE_BASE_MM         8L /* 动态刹车距离基础量：+5cm侧再晚2mm刹车。 */
#define BALL_BRAKE_VEL_GAIN        18L /* 动态刹车距离=基础量+当前速度(mm/20ms)*系数。 */
#define BALL_BRAKE_DIST_MAX_MM     130L /* 动态刹车距离上限，防止噪声速度导致全程反刹。 */
#define BALL_OUTER_BRAKE_DELAY_MM  18L /* 60mm外长行程单独晚一点刹，避免+5到-5过早减速。 */
#define BALL_COAST_MIN_MM          10L /* 已经向目标运动但还没太近时回平，让球滑行。 */
#define BALL_KICK_MIN_STEPS        30L /* 偏离/远离目标时最小启动倾角，用来克服钢球静摩擦。 */
#define BALL_STATIC_KICK_STEPS     42L /* 低速卡住且误差超0.8cm时给更大的破静摩擦启动角。 */
#define BALL_KICK_MAX_STEPS        52L
#define BALL_NEAR_MIN_STEPS        20L /* 10mm内启动角抬高一点，避免近处静摩擦推不动。 */
#define BALL_NEAR_MAX_STEPS        32L
#define BALL_SLOW_MAX_STEPS        32L
#define BALL_BRAKE_MIN_STEPS       32L
#define BALL_NEAR_BRAKE_MIN_STEPS  18L
#define BALL_NEAR_BRAKE_MAX_STEPS  48L
#define BALL_BRAKE_MAX_STEPS       105L
#define BALL_BRAKE_DIST_GAIN       3L /* 越接近目标时，刹车力度线性加大。 */
#define BALL_TARGET_SLEW_STEPS     22L /* 每20ms目标角最多变化步数，兼顾抑制抖动和快速破静摩擦。 */
#define BALL_BRAKE_SLEW_STEPS      48L /* 反向刹车时目标角允许更快切过去，不然刹车命令来得太慢。 */
#define BALL_MID_ZONE_MM           70L /* 中距离限角，远距离才允许打到更大启动角。 */
#define BALL_MID_MAX_STEPS         38L
#define BALL_PID_ZONE_MM           60L /* 60mm内连续PID微调；更远才短推、回平、观察。 */
#define BALL_LONG_KICK_ZONE_MM     45L /* 45mm内也允许短推；该值仅保留为调参参考。 */
#define BALL_KICK_PULSE_TICKS      2U /* 大误差短促给角，避免长距离启动速度过高。 */
#define BALL_KICK_COAST_TICKS      3U /* 推动后至少滑行观察约60ms，避免一直压着越过目标。 */
#define BALL_CLOSE_PULSE_TICKS     1U /* 接近目标只点一下，靠采样结果决定下一次动作。 */
#define BALL_CLOSE_COAST_TICKS     3U
#define BALL_BRAKE_PULSE_TICKS     1U /* 刹车也脉冲化，避免反向调过头。 */
#define BALL_BRAKE_COAST_TICKS     1U
#define BALL_TASK_PID_KP_NUM       12L /* 赛题3独立控制：从0到+5cm，再到-5cm并停住。 */
#define BALL_TASK_PID_KI_NUM       1L
#define BALL_TASK_PID_KD_NUM       10L
#define BALL_TASK_PID_DEN          10L
#define BALL_TASK_PID_I_LIMIT      50L
#define BALL_TASK_BRAKE_BASE_MM    6L
#define BALL_TASK_BRAKE_VEL_GAIN   10L
#define BALL_TASK_BRAKE_MAX_MM     115L
#define BALL_TASK_OUTER_DELAY_MM   18L
#define BALL_TASK_BRAKE_DIST_GAIN  2L
#define BALL_TASK_KICK_MIN_STEPS   30L
#define BALL_TASK_KICK_MAX_STEPS   52L
#define BALL_TASK_MID_MAX_STEPS    38L
#define BALL_TASK_STATIC_KICK_STEPS 48L /* 赛题3卡住时允许大角破静摩擦，靠时间限制避免冲过。 */
#define BALL_TASK_NEAR_STALL_KICK_STEPS 30L /* 终点前1~2.5cm卡住时短促启动，不能再用48步长压。 */
#define BALL_TASK_STATIC_KICK_ZONE_MM BALL_TASK_QUIET_RELEASE_MM /* 超过1cm且卡住就允许大角度启动。 */
#define BALL_TASK_STATIC_RECALL_STEPS 18L /* 静摩擦启动后一拍立刻小反向回调，不等视觉确认。 */
#define BALL_TASK_STATIC_KICK_NEAR_TICKS 6U /* 近距离卡住最多压约120ms，避免3.xcm直接冲过。 */
#define BALL_TASK_STATIC_KICK_MID_TICKS  15U /* 中距离卡住最多压约300ms。 */
#define BALL_TASK_STATIC_KICK_FAR_TICKS  25U /* 远距离卡住最多压约500ms。 */
#define BALL_TASK_QUIET_ZONE_MM    10L /* 赛题3到点安静区：满足1cm要求后停止追小误差。 */
#define BALL_TASK_QUIET_RELEASE_MM 10L /* 赛题要求最大误差1cm，超过10mm必须立刻重新追。 */
#define BALL_TASK_QUIET_SPEED_MM   3L
#define BALL_TASK_SETTLE_ZONE_MM   10L /* 终点±1cm内先回平等停，不急着反向纠偏。 */
#define BALL_TASK_SETTLE_BIAS_STEPS 2L /* 旧固定小偏置仅保留兼容，实际终点保持使用静态平衡曲线。 */
#define BALL_TASK_COAST_MIN_SPEED_MM 1L /* 第三题只要球还在动就回平观察，避免连续压角冲出目标。 */
#define BALL_TASK_REKICK_SPEED_MM  1L /* 放平后速度没起来/停住，且误差>1cm，才允许再次给速度。 */
#define BALL_TASK_START_SPEED_MM   2L /* 启动阶段检测到球已有速度后再放平。 */
#define BALL_TASK_MICRO_ZONE_MM    25L /* 10~25mm只做微调，不允许按远距离大角度启动。 */
#define BALL_TASK_MID_ZONE_MM      60L
#define BALL_TASK_STAGE_IDLE       0U
#define BALL_TASK_STAGE_DRIVE      1U
#define BALL_TASK_STAGE_FLAT       2U
#define BALL_TASK_STAGE_BRAKE      3U
#define BALL_TASK_STAGE_BRAKE_DONE 4U
#define BALL_TASK_DRIVE_PULSE_TICKS 20U /* 远距离最多压400ms；检测到速度会提前放平。 */
#define BALL_TASK_MID_PULSE_TICKS  14U /* 中距离最多压280ms。 */
#define BALL_TASK_MICRO_PULSE_TICKS 8U /* 近目标最多压160ms，避免冲过目标。 */
#define BALL_TASK_DRIVE_COAST_TICKS 10U /* 回平观察约200ms，等球真实响应后再决定下一段。 */
#define BALL_TASK_MICRO_COAST_TICKS 10U
#define BALL_TASK_BRAKE_COAST_TICKS 10U
#define BALL_TASK_NEAR_MIN_STEPS   14L
#define BALL_TASK_NEAR_MAX_STEPS   24L
#define BALL_TASK_MID_MIN_STEPS    26L
#define BALL_TASK_MID_MAX2_STEPS   42L
#define BALL_TASK_POS_FAR_MM       35L /* 0->+5cm单独保守三段，优先不超出。 */
#define BALL_TASK_POS_MID_MM       20L
#define BALL_TASK_POS_FAR_STEPS    28L
#define BALL_TASK_POS_MID_STEPS    22L
#define BALL_TASK_POS_NEAR_STEPS   14L
#define BALL_TASK_POS_BRAKE_BASE_MM 18L
#define BALL_TASK_POS_BRAKE_VEL_GAIN 22L
#define BALL_TASK_NEG_BRAKE_BASE_MM 19L /* +5到-5侧再晚2mm刹车。 */
#define BALL_TASK_NEG_BRAKE_VEL_GAIN 22L
#define BALL_TASK_NEG_FINE_ZONE_MM 25L /* 减完速仍未到位，25mm内只小幅修正，优先不超出。 */
#define BALL_TASK_NEG_NO_LONG_KICK_MM 20L /* 距-5cm小于2cm时，硬禁止大角度长时间破静摩擦。 */
#define BALL_TASK_NEG_FINE_STEPS   14L
#define BALL_TASK_NEG_FINE_BRAKE_MAX 24L
#define BALL_TASK_BRAKE_MIN_STEPS  24L
#define BALL_TASK_BRAKE_MAX_STEPS  86L
#define BALL_TASK_NEAR_BRAKE_MIN   12L
#define BALL_TASK_NEAR_BRAKE_MAX   30L
#define BALL_TASK_SLEW_STEPS       24L
#define BALL_TASK_BRAKE_SLEW_STEPS 52L
#define MOTOR_STEP_FAST_INTERVAL_MS 2U /* 目标步数差较大时的追角速度。 */
#define MOTOR_STEP_SLOW_INTERVAL_MS 5U /* 接近目标角时放慢，减小冲击和抖动。 */
#define MOTOR_STEP_SLOW_BAND       8L
#define MOTOR_DEADBAND_PX          12
#define MOTOR_PID_KP_NUM           30L /* 旧瞄准PID参数暂保留，当前中心稳定不用。 */
#define MOTOR_PID_KI_NUM           1L
#define MOTOR_PID_KD_NUM           15L
#define MOTOR_PID_DEN              100L
#define MOTOR_PID_INTEGRAL_LIMIT   2000L
#define MOTOR_POSITIVE_DIR         0U /* 逻辑正步数=驱动器显示正角度；负向滚球必须走正角度。 */
#define MOTOR_HARD_LIMIT_DEG       40L /* 软件硬限位：以回零点为中心，任何控制都不允许超过正负40度。 */
#define MOTOR_HARD_LIMIT_STEPS     ((int32_t)((MOTOR_STEPS_PER_REV * MOTOR_HARD_LIMIT_DEG + 180L) / 360L))
#define MOTOR_SWING_STEP_INTERVAL_MS 4U /* 摇摆测试改为1ms节拍手动发脉冲，约4ms一步。 */
#define MOTOR_SWING_DWELL_TICKS    15U /* 20ms节拍，端点停约300ms再反向。 */
#define MOTOR_STEP_Pin             GPIO_PIN_4 /* TIM3_CH1/PB4，同时也是STEP输出脚。 */
#define MOTOR_STEP_GPIO_Port       GPIOB
#define MOTOR_OUTPUT_LOCKED_DEFAULT 0U /* 默认不总锁；复位后靠EN和串口命令保持失能。 */
#define MOTOR_EN_DISABLED_LEVEL    GPIO_PIN_SET   /* 张大头/当前接法：EN高电平释放，可手转。 */
#define MOTOR_EN_ENABLED_LEVEL     GPIO_PIN_RESET /* EN低电平使能，确认限位后才允许使用。 */
#define EMM_ADDR_DEFAULT           0x01U
#define EMM_CHECKSUM_FIXED         0x6BU
#define EMM_DISABLE_REPEAT_COUNT   20U /* 复位后约2秒重复失能，避开驱动器上电初始化时间。 */
#define EMM_DISABLE_REPEAT_MS      100U
#define EMM_HOME_DELAY_MS          120U /* K1先使能，等待驱动器锁定后再触发就近回零。 */
#define BALANCE_START_AFTER_HOME_MS 600U /* 回零命令发出后等待机械归零，再自动进入稳定。 */

#define KEY_DEBOUNCE_MS            20U
#define RUNTIME_KEY_K1_Pin         KEY_K1_Pin /* PCB正式按键：K1=PE7，回零后启动中心稳定。 */
#define RUNTIME_KEY_K1_GPIO_Port   KEY_K1_GPIO_Port
#define RUNTIME_KEY_K2_Pin         KEY_K2_Pin /* PCB正式按键：K2=PE9，启动赛题第三问。 */
#define RUNTIME_KEY_K2_GPIO_Port   KEY_K2_GPIO_Port
#define RUNTIME_KEY_K3_Pin         KEY_K3_Pin /* PCB正式按键：K3=PE11，回零后启动纯视觉中心稳定。 */
#define RUNTIME_KEY_K3_GPIO_Port   KEY_K3_GPIO_Port
#define RUNTIME_KEY_K4_Pin         KEY_K4_Pin /* K4=PE13，第一次回零静止，第二次失能释放。 */
#define RUNTIME_KEY_K4_GPIO_Port   KEY_K4_GPIO_Port
#define RUNTIME_KEY_K5_Pin         KEY_K5_Pin /* 旧调试K1改名为K5=PE3，当前暂不分配动作。 */
#define RUNTIME_KEY_K5_GPIO_Port   KEY_K5_GPIO_Port
#define RUNTIME_KEY_K6_Pin         KEY_K6_Pin /* 旧调试K2改名为K6=PC5，当前暂不分配动作。 */
#define RUNTIME_KEY_K6_GPIO_Port   KEY_K6_GPIO_Port
#define K230_TARGET_TIMEOUT_MS     300U

#define MPU6050_ADDR               (0x68U << 1)
#define MPU6050_REG_PWR_MGMT_1     0x6BU
#define MPU6050_REG_ACCEL_XOUT_H   0x3BU
#define ICM42688_ACCEL_FS_MG       4000L /* ICM42688当前配置为±4g，满量程约4000mg */
#define ICM42688_GYRO_FS_DPS       1000L /* ICM42688当前配置为±1000dps。 */
#define ACC_FF_SIGN                (-(BALL_CONTROL_SIGN)) /* 只反901轴向加速度前馈方向，不影响视觉中心外环。 */
#define ACC_FF_DEADBAND_MG         25L /* 静止噪声死区，约0.025g内不让前馈抖动管子。 */
#define ACC_FF_FILTER_NUM          7L /* 加速度前馈低通：7/8旧值+1/8新值，压住901静止细抖。 */
#define ACC_FF_FILTER_DEN          8L
#define ACC_FF_OUTPUT_FILTER_NUM   1L /* 前馈角仅保留一级1/2低通，优先保证响应。 */
#define ACC_FF_OUTPUT_FILTER_DEN   2L
#define ACC_FF_OUTPUT_DEADBAND_STEPS 2L /* 前馈输出小于2步直接归零。 */
#define ACC_FF_GAIN_NUM            5L /* 小车加速度前馈约0.83倍，避免补偿过头。 */
#define ACC_FF_GAIN_DEN            6L
#define ACC_FF_MAX_DEG             14L /* 前馈最多给14度电机角，外环和硬限位仍会统一限幅。 */
#define ACC_FF_SMALL_ANGLE_DEN     6283L /* 水管角theta≈a/g时的弧度到电机步数换算分母2*pi*1000。 */
#define ACC_FF_MAX_VECTOR_MG       20000L /* 加速度合成保护上限，避免静止零点被异常帧误判卡死。 */
#define ACC_FF_MAX_JUMP_MG         800L /* 单帧跳变超过0.8g先当毛刺；连续出现才认为是真实变化。 */
#define ACC_FF_JUMP_ACCEPT_COUNT   3U
#define IMU_LEVEL_ZERO_SAMPLES     50U /* K1回零后取约1秒MS901M数据，把当前状态作为零点。 */
#define IMU_LEVEL_DEADBAND_MG      10L /* 加速度零点附近死区，避免静止抖动引起细碎动作。 */
#define IMU_LEVEL_SIGN             BALL_CONTROL_SIGN /* 保持已调好的MS901M稳定方向，不跟随加速度前馈反向。 */
#define IMU_LEVEL_KI_DEN           300L /* AX积分换算到步数，补偿安装零偏和微小静态误差。 */
#define IMU_LEVEL_I_LIMIT_MG       6000L
#define IMU_LEVEL_KD_NUM           1L /* AX变化率微分，只阻尼轴向加速度变化，不用滚转轴。 */
#define IMU_LEVEL_KD_DEN           5L
#define IMU_LEVEL_SLEW_STEPS       64L /* 每20ms目标角最大变化，缩短前馈到目标角的响应时间。 */
#define IMU_LEVEL_TARGET_DEADBAND_STEPS 5L /* 合成目标角小于5步时当作0，防止静止细碎步进。 */
#define MS901M_Y_TIMEOUT_MS        300U
#define MS901M_CDEG_PER_REV        36000L /* MS901M角度单位为0.01度，360度=36000。 */
#define MS901M_AXIS_ROLL           0U
#define MS901M_AXIS_PITCH          1U
#define MS901M_ACCEL_AXIS_X        0U
#define MS901M_ACCEL_AXIS_Y        1U
#define MS901M_ACCEL_AXIS_Z        2U
#define MS901M_LEVEL_AXIS          MS901M_AXIS_ROLL /* 水管沿901的Y轴，沿管倾斜绕X轴，故用Roll做内环。 */
#define MS901M_ACCEL_FF_AXIS       MS901M_ACCEL_AXIS_X /* 实测小车前后平移加速度对应901的AX，用AX做前馈。 */
#define MS901M_LEVEL_SIGN          (IMU_LEVEL_SIGN) /* 电机正步数使Roll向负方向变化，现场已确认该反馈方向。 */
#define MS901M_LEVEL_GAIN_NUM      3L /* 角度误差到电机补偿的比例，约为完整机构反算增益的一半。 */
#define MS901M_LEVEL_GAIN_DEN      1L
#define MS901M_LEVEL_PIPE_RATIO_DEN 6L /* 机构实测：水管角约为电机角的1/6。 */
#define MS901M_LEVEL_DEADBAND_CD   8L /* Roll误差小于0.08度不再补偿，压住传感器静态细抖。 */
#define MS901M_LEVEL_MAX_CORRECT_STEPS 120L /* 角度内环单次最大附加补偿，防止异常姿态拉满软件限位。 */
#define HOME_AFTER_NONE            0U
#define HOME_AFTER_BALL_CENTER     1U
#define HOME_AFTER_IMU_LEVEL       2U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  uint8_t ok;
  uint32_t frames;
} Mpu6050Data_t;

typedef enum
{
  BALL_TASK_IDLE = 0,
  BALL_TASK_WAIT_HOME,
  BALL_TASK_HOLD_CENTER,
  BALL_TASK_GO_POS,
  BALL_TASK_GO_NEG,
  BALL_TASK_HOLD_NEG,
  BALL_TASK_FINISHED,
  BALL_TASK_DEBUG_FF
} BallTaskState_t;

typedef struct
{
  int32_t pos_q8;
  int32_t last_pos_q8;
  int32_t velocity_q8;
  int32_t pos_integral_q8;
  int32_t velocity_integral_q8;
  int32_t last_velocity_error_q8;
  int32_t output_steps;
  uint8_t valid;
} K3Cascade_t;

static uint8_t g_motor_enabled;
static volatile uint8_t g_motor_output_locked = MOTOR_OUTPUT_LOCKED_DEFAULT;
static uint8_t g_tracking_enabled;
static uint8_t g_swing_test_enabled;
static int8_t g_swing_target_sign;
static uint16_t g_swing_dwell_ticks;
static uint8_t g_swing_step_high;
static uint8_t g_swing_step_wait_ms;
static uint8_t g_swing_dir = 0xFFU;
static uint8_t g_step_gpio_mode;
static uint8_t g_motor_running;
static uint8_t g_motor_dir;
static uint16_t g_motor_rpm;
static int32_t g_motor_pos_steps;
static int32_t g_motor_target_steps;

/* GPIO Feedforward for K3 */
int32_t g_pa8_ff_start_steps = -170L;
int32_t g_pa8_ff_stop_steps = 55L;
int32_t g_pa11_ff_start_steps = -120L;
int32_t g_pa11_ff_stop_steps = 45L;
uint16_t g_pa8_ff_start_duration_ms = 500U;
uint16_t g_pa8_ff_stop_duration_ms = 500U;
uint16_t g_pa11_ff_start_duration_ms = 500U;
uint16_t g_pa11_ff_stop_duration_ms = 500U;
uint8_t g_debug_cursor = 0U;
uint8_t g_debug_page = 0U;

static uint8_t g_pa8_last_state = GPIO_PIN_RESET;
static uint8_t g_pa11_last_state = GPIO_PIN_RESET;
static char g_pa8_edge = '-';
static char g_pa11_edge = '-';
static uint16_t g_pa8_ff_timer = 0U;
static uint16_t g_pa11_ff_timer = 0U;
static int32_t g_pa8_ff_val = 0L;
static int32_t g_pa11_ff_val = 0L;
static uint16_t g_pa8_ff_duration = 0U;
static uint16_t g_pa11_ff_duration = 0U;
static int32_t g_pa8_ff_target_val = 0L;
static int32_t g_pa11_ff_target_val = 0L;

static uint8_t g_balance_step_high;
static uint8_t g_balance_step_wait_ms;
static uint8_t g_balance_dir = 0xFFU;
static int32_t g_pid_integral;
static int16_t g_pid_last_error;
static int32_t g_ball_pid_integral;
static int16_t g_ball_pid_last_error;
static int32_t g_ball_level_pid_integral;
static int16_t g_ball_level_pid_last_error;
static int32_t g_ball_level_offset_steps;
static K3Cascade_t g_k3_cascade;
static K3Cascade_t g_k2_cascade; /* K2赛题3独立串级状态，不能与K3中心稳定共用积分/速度。 */
static int32_t g_ball_task_pid_integral;
static int16_t g_ball_task_pid_last_error;
static int32_t g_ball_pos_filtered_mm;
static int32_t g_ball_pos_last_filtered_mm;
static int32_t g_ball_target_mm = BALL_TARGET_DEFAULT_MM;
static uint8_t g_manual_target_adjust_enabled; /* 仅K1/K3稳定模式允许K5/K6改目标。 */
static uint8_t g_ball_filter_valid;
static uint8_t g_ball_pulse_ticks;
static uint8_t g_ball_pulse_coast_ticks;
static uint8_t g_ball_pulse_coast_reload;
static int32_t g_ball_pulse_control_steps;
static uint8_t g_ball_task_static_recall_ticks;
static int32_t g_ball_task_static_recall_steps;
static uint8_t g_ball_task_static_kick_ticks;
static int32_t g_ball_task_static_kick_steps;
static uint8_t g_ball_task_quiet_latched;
static uint8_t g_ball_task_stage;
static uint8_t g_ball_task_stage_ticks;
static uint8_t g_ball_task_flat_reload_ticks;
static int32_t g_ball_task_stage_steps;

static int16_t g_icm_axis_acc_mg;   /* 管子沿X轴，AX作为轴向加速度。 */
static int16_t g_icm_axis_acc_filtered_mg;
static int16_t g_ms901m_axis_acc_mg; /* MS901M选定轴向加速度，单位mg。 */
static int16_t g_ms901m_axis_acc_filtered_mg;
static int16_t g_ms901m_acc_error_mg;
static int32_t g_accel_ff_steps;
static int16_t g_accel_ff_zero_mg;
static int32_t g_accel_ff_zero_sum;
static uint16_t g_accel_ff_zero_count;
static uint8_t g_accel_ff_zero_valid;
static uint8_t g_icm_ok;
static uint8_t g_icm_acc_filter_valid;
static uint8_t g_icm_acc_jump_count;
static uint8_t g_ms901m_acc_ok;
static uint8_t g_ms901m_acc_filter_valid;
static uint8_t g_ms901m_acc_jump_count;
static uint8_t g_imu_level_active;
static uint8_t g_imu_zero_pending;
static uint16_t g_imu_zero_count;
static int32_t g_imu_zero_acc_sum;
static int16_t g_imu_acc_zero_mg;
static int32_t g_imu_level_integral_mg;
static int16_t g_imu_level_last_error_mg;
static int32_t g_ms901m_level_zero_cd;
static int32_t g_ms901m_level_zero_sum_cd;
static int32_t g_ms901m_level_error_cd;
static uint8_t g_ms901m_level_zero_valid;

static Mpu6050Data_t g_mpu6050;
static volatile uint8_t g_mpu_read_request;
static volatile uint8_t g_oled_update_request;
static volatile uint8_t g_motor_disable_request;
static uint8_t g_motor_disable_pending;
static uint32_t g_motor_disable_last_tx_tick;
static uint32_t g_motor_disable_sent_count;
static volatile uint8_t g_motor_home_request;
static uint8_t g_motor_home_pending;
static uint32_t g_motor_home_tick;
static uint32_t g_motor_home_sent_count;
static uint8_t g_balance_start_after_home_pending;
static uint32_t g_balance_start_after_home_tick;
static uint8_t g_after_home_mode;
static uint8_t g_k4_next_disable; /* K4两态开关：0=下次回零，1=下次失能。 */
static BallTaskState_t g_ball_task_state = BALL_TASK_IDLE;
static uint32_t g_ball_task_start_tick;
static int32_t g_k2_center_mm = 0;
  int32_t g_k2_offset_go_pos = -15L;
  int32_t g_k2_offset_go_neg = -20L;
  int32_t g_k3_offset_10cm = 0L;
  static uint16_t g_ball_task_arrive_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void App_Process1ms(void);
static void App_Background(void);
static void Tracking_StartAtTarget(int32_t target_mm);
static void BallTarget_AdjustByMm(int32_t delta_mm);
static void BallTask_RequestCenterHome(void);
static void BallTask_StartCenterAfterHome(void);
static void BallTask_StartSequence(void);
static void HomeOnly_Request(void);
static uint8_t BallTask_Update20ms(int32_t abs_error, int32_t abs_velocity);
static void BallTask_CascadeControl20ms(int32_t raw_pos_mm);
static uint8_t BallTask_IsSequenceState(void);
static int32_t BallTask_GetSettleBiasSteps(void);
static void BallTask_Control20ms(int16_t error, int32_t abs_error, int32_t velocity,
                                 int32_t abs_velocity, uint8_t moving_toward_target,
                                 uint8_t predicted_cross_target);
static void BallLevel_Reset(void);
static int32_t BallLevel_Process20ms(void);
static void K3Cascade_Reset(K3Cascade_t *cascade);
static int32_t K3Cascade_Process20ms(K3Cascade_t *cascade, int32_t raw_pos_mm,
                                     int32_t target_mm);
static void Ball_ResetPulseControl(void);
static void App_UpdateIcmSnapshot(void);
static int32_t App_GetMs901mLevelAxisCd(void);
static int16_t App_GetMs901mAccelAxisMg(const IMU901_Data_t *imu);
static void App_UpdateMs901mAccelSnapshot(void);
static int32_t App_Ms901mAngleDiffCd(int32_t current_cd, int32_t zero_cd);
static int32_t ImuLevel_ApplyMs901mAngleFeedback(int32_t outer_target_steps);
static int32_t App_AddAccelFeedforward(int32_t control_steps);
static void Oled_ShowLine(uint8_t row, const char *text);
static void ImuLevel_RequestHome(void);
static void ImuLevel_StartZeroAfterHome(void);
static uint8_t ImuLevel_Process20ms(void);
static void ImuLevel_Reset(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int16_t App_ReadInt16BE(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint8_t App_ReadKeyPress(GPIO_TypeDef *port, uint16_t pin, uint8_t *stable_level,
                                uint8_t *last_sample, uint16_t *debounce_ms)
{
  uint8_t sample = (uint8_t)HAL_GPIO_ReadPin(port, pin);

  if (sample != *last_sample)
  {
    *last_sample = sample;
    *debounce_ms = 0U;
    return 0U;
  }

  if (*debounce_ms < KEY_DEBOUNCE_MS)
  {
    (*debounce_ms)++;
    return 0U;
  }

  if (sample != *stable_level)
  {
    *stable_level = sample;
    return (*stable_level == GPIO_PIN_RESET) ? 1U : 0U;
  }

  return 0U;
}

static void App_InitRuntimeKeys(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* 不改.ioc：当前调试板仍使用旧按键PE3/PC5，按下为低电平。 */
  GPIO_InitStruct.Pin = RUNTIME_KEY_K1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RUNTIME_KEY_K1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RUNTIME_KEY_K2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RUNTIME_KEY_K2_GPIO_Port, &GPIO_InitStruct);
}

static void Motor_ConfigStepGpio(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (g_step_gpio_mode != 0U)
  {
    return;
  }

  (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = MOTOR_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MOTOR_STEP_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
  g_step_gpio_mode = 1U;
}

static void Motor_ConfigStepPwm(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (g_step_gpio_mode == 0U)
  {
    return;
  }

  HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = MOTOR_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(MOTOR_STEP_GPIO_Port, &GPIO_InitStruct);
  g_step_gpio_mode = 0U;
}

static void Motor_Enable(uint8_t enable)
{
  if (g_motor_output_locked != 0U)
  {
    (void)enable;
    g_motor_enabled = 0U;
    g_motor_running = 0U;
    g_motor_rpm = 0U;
    (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, MOTOR_EN_DISABLED_LEVEL);
    return;
  }

  g_motor_enabled = (enable != 0U) ? 1U : 0U;
  HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin,
                    (g_motor_enabled != 0U) ? MOTOR_EN_ENABLED_LEVEL : MOTOR_EN_DISABLED_LEVEL);
}

static void Motor_Stop(void)
{
  if (g_motor_running != 0U)
  {
    (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  }
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  if (g_step_gpio_mode != 0U)
  {
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
  }
  g_swing_step_high = 0U;
  g_swing_step_wait_ms = 0U;
  g_balance_step_high = 0U;
  g_balance_step_wait_ms = 0U;
  g_motor_running = 0U;
  g_motor_rpm = 0U;
}

static int8_t Motor_DirSign(uint8_t dir)
{
  return (dir == MOTOR_POSITIVE_DIR) ? 1 : -1;
}

static void Motor_UpdateEstimatedPosition20ms(void)
{
  uint32_t step_delta;

  if (g_motor_running == 0U)
  {
    return;
  }

  /* 开环估算：rpm * steps/rev * 20ms / 60000ms，作为机械保护用。 */
  step_delta = ((uint32_t)g_motor_rpm * MOTOR_STEPS_PER_REV + 1500UL) / 3000UL;
  if (step_delta == 0UL)
  {
    step_delta = 1UL;
  }

  g_motor_pos_steps += (int32_t)Motor_DirSign(g_motor_dir) * (int32_t)step_delta;
  if (g_motor_pos_steps > MOTOR_HARD_LIMIT_STEPS)
  {
    g_motor_pos_steps = MOTOR_HARD_LIMIT_STEPS;
  }
  else if (g_motor_pos_steps < -MOTOR_HARD_LIMIT_STEPS)
  {
    g_motor_pos_steps = -MOTOR_HARD_LIMIT_STEPS;
  }
}

static uint8_t Motor_IsDirAllowed(uint8_t dir)
{
  if ((Motor_DirSign(dir) > 0) && (g_motor_pos_steps >= MOTOR_HARD_LIMIT_STEPS))
  {
    return 0U;
  }
  if ((Motor_DirSign(dir) < 0) && (g_motor_pos_steps <= -MOTOR_HARD_LIMIT_STEPS))
  {
    return 0U;
  }
  return 1U;
}

static uint16_t Motor_ClampRpmByHardLimit(uint8_t dir, uint16_t rpm)
{
  int32_t remain_steps;
  uint32_t max_rpm;

  if (Motor_DirSign(dir) > 0)
  {
    remain_steps = MOTOR_HARD_LIMIT_STEPS - g_motor_pos_steps;
  }
  else
  {
    remain_steps = g_motor_pos_steps + MOTOR_HARD_LIMIT_STEPS;
  }

  if (remain_steps <= 0L)
  {
    return 0U;
  }

  /* 保证下一个20ms控制周期内不会跨过正负12度软件硬限位。 */
  max_rpm = ((uint32_t)remain_steps * 3000UL) / MOTOR_STEPS_PER_REV;
  if (max_rpm < (uint32_t)rpm)
  {
    rpm = (uint16_t)max_rpm;
  }

  return rpm;
}

static HAL_StatusTypeDef Motor_SendSerialEnable(uint8_t enable)
{
  uint8_t cmd[] = {
    EMM_ADDR_DEFAULT,
    0xF3U,
    0xABU,
    (enable != 0U) ? 0x01U : 0x00U,
    0x00U,
    EMM_CHECKSUM_FIXED
  };

  /* 张大头Emm V5官方帧：01 F3 AB 00/01 00 6B，USART3接驱动器。 */
  return HAL_UART_Transmit(&huart3, cmd, (uint16_t)sizeof(cmd), 20U);
}

static HAL_StatusTypeDef Motor_SendSerialHome(void)
{
  uint8_t cmd[] = {
    EMM_ADDR_DEFAULT,
    0x9AU,
    0x00U,
    0x00U,
    EMM_CHECKSUM_FIXED
  };

  /* 张大头Emm V5官方帧：01 9A 00 00 6B，00为单圈就近回零。 */
  return HAL_UART_Transmit(&huart3, cmd, (uint16_t)sizeof(cmd), 20U);
}

static void Motor_RequestSerialDisable(void)
{
  Motor_Stop();
  g_tracking_enabled = 0U;
  g_manual_target_adjust_enabled = 0U;
  g_swing_test_enabled = 0U;
  g_swing_dir = 0xFFU;
  g_balance_dir = 0xFFU;
  g_balance_start_after_home_pending = 0U;
  g_after_home_mode = HOME_AFTER_NONE;
  g_motor_home_request = 0U;
  g_motor_home_pending = 0U;
  g_ball_task_state = BALL_TASK_IDLE;
  g_ball_task_arrive_ms = 0U;
  Ball_ResetPulseControl();
  ImuLevel_Reset();
  g_motor_enabled = 0U;
  g_motor_target_steps = 0L;
  HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, MOTOR_EN_DISABLED_LEVEL);
  g_motor_disable_pending = EMM_DISABLE_REPEAT_COUNT;
  g_motor_disable_last_tx_tick = 0U;
}

static void Motor_RequestHome(void)
{
  Motor_Stop();
  g_tracking_enabled = 0U;
  g_swing_test_enabled = 0U;
  g_swing_target_sign = 1;
  g_swing_dwell_ticks = 0U;
  g_swing_dir = 0xFFU;
  g_balance_dir = 0xFFU;
  g_pid_integral = 0L;
  g_pid_last_error = 0;
  Ball_ResetPulseControl();
  g_motor_pos_steps = 0L;
  g_motor_target_steps = 0L;
  g_motor_output_locked = 0U;
  g_motor_disable_request = 0U;
  g_motor_disable_pending = 0U;
  g_motor_disable_last_tx_tick = 0U;
  Motor_Enable(1U);
  (void)Motor_SendSerialEnable(1U);
  g_motor_home_tick = HAL_GetTick();
  g_motor_home_pending = 1U;
}

static void Motor_RunSpeed(uint8_t dir, uint16_t rpm)
{
  uint32_t pps;
  uint32_t period;

  Motor_ConfigStepPwm();

  if (g_motor_output_locked != 0U)
  {
    (void)dir;
    (void)rpm;
    Motor_Stop();
    HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, MOTOR_EN_DISABLED_LEVEL);
    g_motor_enabled = 0U;
    return;
  }

  if ((g_motor_enabled == 0U) || (rpm < MOTOR_MIN_RPM))
  {
    Motor_Stop();
    return;
  }
  if (Motor_IsDirAllowed(dir) == 0U)
  {
    g_pid_integral = 0L;
    g_pid_last_error = 0;
    Motor_Stop();
    return;
  }
  if (rpm > MOTOR_MAX_RPM)
  {
    rpm = MOTOR_MAX_RPM;
  }
  rpm = Motor_ClampRpmByHardLimit(dir, rpm);
  if (rpm < MOTOR_MIN_RPM)
  {
    g_pid_integral = 0L;
    g_pid_last_error = 0;
    Motor_Stop();
    return;
  }

  pps = ((uint32_t)rpm * MOTOR_STEPS_PER_REV + 30UL) / 60UL;
  if (pps == 0UL)
  {
    Motor_Stop();
    return;
  }

  period = MOTOR_TIMER_CLOCK_HZ / pps;
  if (period < 2UL)
  {
    period = 2UL;
  }
  if (period > 65536UL)
  {
    period = 65536UL;
  }

  HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, (dir == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  __HAL_TIM_DISABLE(&htim3);
  __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1UL);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, period / 2UL);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_ENABLE(&htim3);

  if (g_motor_running == 0U)
  {
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  }
  g_motor_running = 1U;
  g_motor_dir = dir;
  g_motor_rpm = rpm;
}

static void Tracking_ResetPid(void)
{
  g_pid_integral = 0L;
  g_pid_last_error = 0;
  g_ball_pid_integral = 0L;
  g_ball_pid_last_error = 0;
  BallLevel_Reset();
  K3Cascade_Reset(&g_k3_cascade);
  K3Cascade_Reset(&g_k2_cascade);
  g_ball_task_pid_integral = 0L;
  g_ball_task_pid_last_error = 0;
  g_ball_pos_filtered_mm = 0L;
  g_ball_pos_last_filtered_mm = 0L;
  g_ball_filter_valid = 0U;
  Ball_ResetPulseControl();
}

static void Ball_ResetPulseControl(void)
{
  g_ball_pulse_ticks = 0U;
  g_ball_pulse_coast_ticks = 0U;
  g_ball_pulse_coast_reload = 0U;
  g_ball_pulse_control_steps = 0L;
  g_ball_task_static_recall_ticks = 0U;
  g_ball_task_static_recall_steps = 0L;
  g_ball_task_static_kick_ticks = 0U;
  g_ball_task_static_kick_steps = 0L;
  g_ball_task_quiet_latched = 0U;
  g_ball_task_stage = BALL_TASK_STAGE_IDLE;
  g_ball_task_stage_ticks = 0U;
  g_ball_task_flat_reload_ticks = 0U;
  g_ball_task_stage_steps = 0L;
}

static void BallLevel_Reset(void)
{
  g_ball_level_pid_integral = 0L;
  g_ball_level_pid_last_error = 0;
  g_ball_level_offset_steps = 0L;
}

static void Ball_ResetPidOnly(void)
{
  g_pid_integral = 0L;
  g_pid_last_error = 0;
  g_ball_pid_integral = 0L;
  BallLevel_Reset();
  g_ball_task_pid_integral = 0L;
  if (g_ball_filter_valid != 0U)
  {
    g_ball_pid_last_error = (int16_t)(g_ball_target_mm - g_ball_pos_filtered_mm);
    g_ball_task_pid_last_error = g_ball_pid_last_error;
  }
  else
  {
    g_ball_pid_last_error = 0;
    g_ball_task_pid_last_error = 0;
  }
  Ball_ResetPulseControl();
}

static int32_t BallLevel_Process20ms(void)
{
#if (BALL_LEVEL_ENABLE == 0U)
  BallLevel_Reset();
  return 0L;
#else
  int32_t raw_pos;
  int32_t velocity;
  int32_t error;
  int32_t derivative;
  int32_t control;
  int32_t abs_error;
  int32_t abs_velocity;
  int32_t abs_control;
  int32_t drive_sign;
  int32_t predicted_error;
  int32_t predicted_abs_error;
  int32_t delta_steps;
  int32_t slew_steps;
  uint8_t moving_toward_target;
  uint8_t vision_available;

  vision_available = AppK230_HasFreshTarget(K230_TARGET_TIMEOUT_MS);
  if (vision_available == 0U)
  {
    g_ball_level_pid_integral = 0L;
    g_ball_level_pid_last_error = 0;
    if (g_ball_level_offset_steps > BALL_LEVEL_SLEW_STEPS)
    {
      g_ball_level_offset_steps -= BALL_LEVEL_SLEW_STEPS;
    }
    else if (g_ball_level_offset_steps < -BALL_LEVEL_SLEW_STEPS)
    {
      g_ball_level_offset_steps += BALL_LEVEL_SLEW_STEPS;
    }
    else
    {
      g_ball_level_offset_steps = 0L;
    }
    return g_ball_level_offset_steps;
  }

  raw_pos = (int32_t)AppK230_GetPosMm();
  if (g_ball_filter_valid == 0U)
  {
    g_ball_pos_filtered_mm = raw_pos;
    g_ball_pos_last_filtered_mm = raw_pos;
    g_ball_filter_valid = 1U;
  }
  else
  {
    g_ball_pos_last_filtered_mm = g_ball_pos_filtered_mm;
    g_ball_pos_filtered_mm =
      ((g_ball_pos_filtered_mm * BALL_FILTER_NUM) + raw_pos) / BALL_FILTER_DEN;
  }

  velocity = g_ball_pos_filtered_mm - g_ball_pos_last_filtered_mm;
  error = (int32_t)g_ball_target_mm - g_ball_pos_filtered_mm;
  predicted_error = error - (velocity * BALL_LEVEL_PREDICT_CYCLES);
  abs_error = (error >= 0L) ? error : -error;
  abs_velocity = (velocity >= 0L) ? velocity : -velocity;
  predicted_abs_error = (predicted_error >= 0L) ? predicted_error : -predicted_error;
  moving_toward_target = ((error * velocity) > 0L) ? 1U : 0U;
  drive_sign = (BALL_CONTROL_SIGN * error >= 0L) ? 1L : -1L;

  if ((abs_error <= BALL_LEVEL_OK_ZONE_MM) && (abs_velocity <= BALL_LEVEL_HOLD_SPEED_MM))
  {
    g_ball_level_pid_integral = 0L;
    g_ball_level_pid_last_error = (int16_t)error;
    control = 0L;
  }
  else if (abs_error <= BALL_LEVEL_DAMP_ZONE_MM)
  {
    g_ball_level_pid_integral = 0L;
    g_ball_level_pid_last_error = (int16_t)error;

    if (abs_velocity <= BALL_LEVEL_HOLD_SPEED_MM)
    {
      control = 0L;
    }
    else
    {
      if (error == 0L)
      {
        drive_sign = ((BALL_CONTROL_SIGN * -velocity) >= 0L) ? 1L : -1L;
      }

      /* 近中心只做阻尼：往外跑小拉回，往中心跑小反偏刹车，不追3~4mm视觉噪声。 */
      control = (moving_toward_target != 0U) ?
                (-drive_sign * BALL_LEVEL_DAMP_BRAKE_STEPS) :
                (drive_sign * BALL_LEVEL_DAMP_BIAS_STEPS);
    }
  }
  else
  {
    g_ball_level_pid_integral += error;
    if (g_ball_level_pid_integral > BALL_LEVEL_PID_I_LIMIT)
    {
      g_ball_level_pid_integral = BALL_LEVEL_PID_I_LIMIT;
    }
    else if (g_ball_level_pid_integral < -BALL_LEVEL_PID_I_LIMIT)
    {
      g_ball_level_pid_integral = -BALL_LEVEL_PID_I_LIMIT;
    }

    derivative = error - (int32_t)g_ball_level_pid_last_error;
    g_ball_level_pid_last_error = (int16_t)error;

    /* 定点稳定只用连续线性PD，不进入第三问那种短推/回平脉冲状态机。 */
    control = BALL_CONTROL_SIGN *
              (((BALL_LEVEL_PID_KP_NUM * error) +
                (BALL_LEVEL_PID_KI_NUM * g_ball_level_pid_integral) +
                (BALL_LEVEL_PID_KD_NUM * derivative)) / BALL_LEVEL_PID_DEN);

    abs_control = (control >= 0L) ? control : -control;

    if ((moving_toward_target != 0U) &&
        ((abs_error >= BALL_LEVEL_GUARD_MM) || (predicted_abs_error >= BALL_LEVEL_LIMIT_MM)))
    {
      abs_control = BALL_LEVEL_BRAKE_MIN_STEPS +
                    ((predicted_abs_error > BALL_LEVEL_LIMIT_MM) ?
                     ((predicted_abs_error - BALL_LEVEL_LIMIT_MM) * BALL_LEVEL_BRAKE_GAIN) : 0L);
      if (abs_control > BALL_LEVEL_BRAKE_MAX_STEPS)
      {
        abs_control = BALL_LEVEL_BRAKE_MAX_STEPS;
      }
      drive_sign = -drive_sign;
    }
    else
    {
      if ((abs_error >= BALL_LEVEL_GUARD_MM) && (abs_control < BALL_LEVEL_GUARD_MIN_STEPS))
      {
        abs_control = BALL_LEVEL_GUARD_MIN_STEPS;
      }
      else if ((abs_error >= BALL_LEVEL_RELEASE_MM) &&
               (abs_velocity <= BALL_LEVEL_HOLD_SPEED_MM) &&
               (abs_control < BALL_LEVEL_STATIC_MIN_STEPS))
      {
        abs_control = BALL_LEVEL_STATIC_MIN_STEPS;
      }
    }

    if (abs_error <= BALL_LEVEL_RELEASE_MM)
    {
      if (abs_control > BALL_LEVEL_NEAR_MAX_STEPS)
      {
        abs_control = BALL_LEVEL_NEAR_MAX_STEPS;
      }
    }
    else if (abs_control > BALL_LEVEL_MAX_STEPS)
    {
      abs_control = BALL_LEVEL_MAX_STEPS;
    }

    control = drive_sign * abs_control;
  }

  delta_steps = control - g_ball_level_offset_steps;
  slew_steps = (((control > 0L) && (g_ball_level_offset_steps < 0L)) ||
                ((control < 0L) && (g_ball_level_offset_steps > 0L))) ?
               BALL_LEVEL_BRAKE_SLEW_STEPS : BALL_LEVEL_SLEW_STEPS;
  if (delta_steps > slew_steps)
  {
    control = g_ball_level_offset_steps + slew_steps;
  }
  else if (delta_steps < -slew_steps)
  {
    control = g_ball_level_offset_steps - slew_steps;
  }

  g_ball_level_offset_steps = control;
  return g_ball_level_offset_steps;
#endif
}

static void K3Cascade_Reset(K3Cascade_t *cascade)
{
  cascade->pos_q8 = 0L;
  cascade->last_pos_q8 = 0L;
  cascade->velocity_q8 = 0L;
  cascade->pos_integral_q8 = 0L;
  cascade->velocity_integral_q8 = 0L;
  cascade->last_velocity_error_q8 = 0L;
  cascade->output_steps = 0L;
  cascade->valid = 0U;
}

static int32_t K3Cascade_Process20ms(K3Cascade_t *cascade, int32_t raw_pos_mm,
                                     int32_t target_mm)
{
  int32_t raw_pos_q8;
  int32_t pos_error_q8;
  int32_t abs_pos_error_q8;
  int32_t velocity_ref_q8;
  int32_t velocity_error_q8;
  int32_t velocity_derivative_q8;
  int32_t control_steps;
  int32_t abs_control_steps;
  int32_t delta_steps;
  int32_t slew_steps;
  int32_t control_limit_steps;
  int32_t approach_speed_limit_q8;
  uint8_t braking;

  raw_pos_q8 = raw_pos_mm * K3_CASCADE_Q8;
  if (cascade->valid == 0U)
  {
    cascade->pos_q8 = raw_pos_q8;
    cascade->last_pos_q8 = raw_pos_q8;
    cascade->velocity_q8 = 0L;
    cascade->valid = 1U;
    return 0L;
  }

  cascade->last_pos_q8 = cascade->pos_q8;
  cascade->pos_q8 =
    ((cascade->pos_q8 * BALL_FILTER_NUM) + raw_pos_q8) / BALL_FILTER_DEN;
  cascade->velocity_q8 = cascade->pos_q8 - cascade->last_pos_q8;
  pos_error_q8 = (target_mm * K3_CASCADE_Q8) - cascade->pos_q8;
  abs_pos_error_q8 = (pos_error_q8 >= 0L) ? pos_error_q8 : -pos_error_q8;

  if (abs_pos_error_q8 <= K3_CASCADE_POS_DEADBAND_Q8)
  {
    /* 到当前目标后位置环给零速，内环只负责把剩余球速刹掉。 */
    pos_error_q8 = 0L;
    cascade->pos_integral_q8 = 0L;
  }
  else
  {
    cascade->pos_integral_q8 += pos_error_q8;
    if (cascade->pos_integral_q8 > K3_CASCADE_POS_I_LIMIT_Q8)
    {
      cascade->pos_integral_q8 = K3_CASCADE_POS_I_LIMIT_Q8;
    }
    else if (cascade->pos_integral_q8 < -K3_CASCADE_POS_I_LIMIT_Q8)
    {
      cascade->pos_integral_q8 = -K3_CASCADE_POS_I_LIMIT_Q8;
    }
  }

  /* 外环PI：位置误差只生成期望球速度，单位为mm/20ms的Q8定点数。 */
  velocity_ref_q8 =
    (K3_CASCADE_POS_KP_NUM * pos_error_q8) / K3_CASCADE_POS_KP_DEN;
  velocity_ref_q8 +=
    (K3_CASCADE_POS_KI_NUM * cascade->pos_integral_q8) / K3_CASCADE_POS_KI_DEN;
  if (velocity_ref_q8 > K3_CASCADE_VREF_MAX_Q8)
  {
    velocity_ref_q8 = K3_CASCADE_VREF_MAX_Q8;
  }
  else if (velocity_ref_q8 < -K3_CASCADE_VREF_MAX_Q8)
  {
    velocity_ref_q8 = -K3_CASCADE_VREF_MAX_Q8;
  }

  /*
   * 向当前目标滚动时，先用正常给定把球脱离静摩擦；一旦已建立朝目标的球速，
   * 在目标35mm内按剩余距离逐步降低目标球速。这样速度环会在到位前主动刹车，
   * 而不是等球越过目标后才反向拉回。
   */
  if ((abs_pos_error_q8 < K3_CASCADE_CENTER_BRAKE_START_Q8) &&
      (((pos_error_q8 > 0L) &&
        (cascade->velocity_q8 > K3_CASCADE_HOLD_SPEED_Q8)) ||
       ((pos_error_q8 < 0L) &&
        (cascade->velocity_q8 < -K3_CASCADE_HOLD_SPEED_Q8))))
  {
    approach_speed_limit_q8 =
      (abs_pos_error_q8 * K3_CASCADE_VREF_MAX_Q8) /
      K3_CASCADE_CENTER_BRAKE_START_Q8;
    if (velocity_ref_q8 > approach_speed_limit_q8)
    {
      velocity_ref_q8 = approach_speed_limit_q8;
    }
    else if (velocity_ref_q8 < -approach_speed_limit_q8)
    {
      velocity_ref_q8 = -approach_speed_limit_q8;
    }
  }

  velocity_error_q8 = velocity_ref_q8 - cascade->velocity_q8;
  if ((pos_error_q8 == 0L) &&
      (cascade->velocity_q8 <= K3_CASCADE_HOLD_SPEED_Q8) &&
      (cascade->velocity_q8 >= -K3_CASCADE_HOLD_SPEED_Q8))
  {
    cascade->velocity_integral_q8 = 0L;
    cascade->last_velocity_error_q8 = 0L;
    control_steps = 0L;
  }
  else
  {
    cascade->velocity_integral_q8 += velocity_error_q8;
    if (cascade->velocity_integral_q8 > K3_CASCADE_VEL_I_LIMIT_Q8)
    {
      cascade->velocity_integral_q8 = K3_CASCADE_VEL_I_LIMIT_Q8;
    }
    else if (cascade->velocity_integral_q8 < -K3_CASCADE_VEL_I_LIMIT_Q8)
    {
      cascade->velocity_integral_q8 = -K3_CASCADE_VEL_I_LIMIT_Q8;
    }

    velocity_derivative_q8 = velocity_error_q8 - cascade->last_velocity_error_q8;
    cascade->last_velocity_error_q8 = velocity_error_q8;

    /* 内环PID：速度误差直接换成杆的目标角度（电机步数）。 */
    control_steps =
      (K3_CASCADE_VEL_KP_NUM * velocity_error_q8) / K3_CASCADE_VEL_KP_DEN;
    control_steps +=
      (K3_CASCADE_VEL_KI_NUM * cascade->velocity_integral_q8) / K3_CASCADE_VEL_KI_DEN;
    control_steps +=
      (K3_CASCADE_VEL_KD_NUM * velocity_derivative_q8) / K3_CASCADE_VEL_KD_DEN;
    control_steps *= BALL_CONTROL_SIGN;

    /* 控制角和球速同号时，杆的加速度与当前滚球方向相反，属于减速调节。 */
    braking = (((cascade->velocity_q8 > K3_CASCADE_HOLD_SPEED_Q8) &&
                (control_steps > 0L)) ||
               ((cascade->velocity_q8 < -K3_CASCADE_HOLD_SPEED_Q8) &&
                (control_steps < 0L))) ? 1U : 0U;
    if (abs_pos_error_q8 <= (4L * K3_CASCADE_Q8))
    {
      if (braking != 0U)
      {
        control_limit_steps = (control_steps < 0L) ?
                              K3_CASCADE_NEAR_NEG_BRAKE_MAX_STEPS :
                              K3_CASCADE_NEAR_POS_BRAKE_MAX_STEPS;
      }
      else
      {
        control_limit_steps = (control_steps < 0L) ?
                              K3_CASCADE_NEAR_NEG_MAX_STEPS : K3_CASCADE_NEAR_POS_MAX_STEPS;
      }
    }
    else
    {
      if (braking != 0U)
      {
        control_limit_steps = (control_steps < 0L) ?
                              K3_CASCADE_NEG_BRAKE_MAX_STEPS : K3_CASCADE_POS_BRAKE_MAX_STEPS;
      }
      else
      {
        control_limit_steps = (control_steps < 0L) ?
                              K3_CASCADE_NEG_MAX_STEPS : K3_CASCADE_POS_MAX_STEPS;
      }
    }
    abs_control_steps = (control_steps >= 0L) ? control_steps : -control_steps;
    if (abs_control_steps > control_limit_steps)
    {
      control_steps = (control_steps >= 0L) ? control_limit_steps : -control_limit_steps;
    }
  }

  delta_steps = control_steps - cascade->output_steps;
  slew_steps = (((control_steps > 0L) && (cascade->output_steps < 0L)) ||
                ((control_steps < 0L) && (cascade->output_steps > 0L))) ?
               K3_CASCADE_BRAKE_SLEW_STEPS : K3_CASCADE_SLEW_STEPS;
  if (delta_steps > slew_steps)
  {
    control_steps = cascade->output_steps + slew_steps;
  }
  else if (delta_steps < -slew_steps)
  {
    control_steps = cascade->output_steps - slew_steps;
  }

  cascade->output_steps = control_steps;
  return control_steps;
}

static void Ball_SetTarget(int32_t target_mm)
{
  g_ball_target_mm = target_mm;
  if (g_ball_target_mm > BALL_TARGET_MAX_MM)
  {
    g_ball_target_mm = BALL_TARGET_MAX_MM;
  }
  else if (g_ball_target_mm < BALL_TARGET_MIN_MM)
  {
    g_ball_target_mm = BALL_TARGET_MIN_MM;
  }

  Ball_ResetPidOnly();
}

static void BallTarget_AdjustByMm(int32_t delta_mm)
{
  if (g_manual_target_adjust_enabled == 0U)
  {
    return;
  }

  /* K2第三问的目标和K1/K3手调稳定目标相互独立，不能被K5/K6改写。 */
  if ((BallTask_IsSequenceState() != 0U) ||
      (g_ball_task_state == BALL_TASK_FINISHED))
  {
    return;
  }

  Ball_SetTarget(g_ball_target_mm + delta_mm);
  g_oled_update_request = 1U;
}

static void BallTask_RequestCenterHome(void)
{
  /* Lock removed so K3 can interrupt K2 at any time */

  /* K3：只用K230球位置做中心稳定，明确关闭MS901M清零/前馈链路。 */
  Tracking_ResetPid();
  ImuLevel_Reset();
  g_tracking_enabled = 0U;
  g_swing_test_enabled = 0U;
  g_swing_dir = 0xFFU;
  g_balance_dir = 0xFFU;
  g_ball_target_mm = BALL_TARGET_DEFAULT_MM;
  g_manual_target_adjust_enabled = 1U;
  g_ball_filter_valid = 0U;
  BallLevel_Reset();
  g_ball_task_state = BALL_TASK_WAIT_HOME;
  g_ball_task_arrive_ms = 0U;
  g_after_home_mode = HOME_AFTER_BALL_CENTER;
  g_balance_start_after_home_pending = 1U;
  g_motor_home_request = 1U;
}

static void BallTask_StartCenterAfterHome(void)
{
  g_ball_task_state = BALL_TASK_HOLD_CENTER;
  g_ball_task_arrive_ms = 0U;
  Tracking_StartAtTarget(g_ball_target_mm);
}

static void BallTask_StartSequence(void)
{
  if ((g_tracking_enabled == 0U) ||
      (g_ball_task_state == BALL_TASK_WAIT_HOME))
  {
    return;
  }

  g_k2_center_mm = g_ball_target_mm;

  ImuLevel_Reset();
  g_manual_target_adjust_enabled = 0U;
  g_ball_task_state = BALL_TASK_GO_POS;
  g_ball_task_start_tick = HAL_GetTick();
  g_ball_task_arrive_ms = 0U;
  Ball_SetTarget(g_k2_center_mm + BALL_TASK_POS_TARGET_MM);
  K3Cascade_Reset(&g_k2_cascade);
  g_motor_target_steps = 0L;
}

static void HomeOnly_Request(void)
{
  /* K4第一下：只回机械零点，不启动IMU稳定或赛题第三问。 */
  Tracking_ResetPid();
  ImuLevel_Reset();
  g_tracking_enabled = 0U;
  g_swing_test_enabled = 0U;
  g_swing_dir = 0xFFU;
  g_balance_dir = 0xFFU;
  g_ball_task_state = BALL_TASK_IDLE;
  g_manual_target_adjust_enabled = 0U;
  g_ball_task_arrive_ms = 0U;
  g_ball_target_mm = BALL_TARGET_DEFAULT_MM;
  g_ball_filter_valid = 0U;
  g_after_home_mode = HOME_AFTER_NONE;
  g_balance_start_after_home_pending = 0U;
  g_motor_home_request = 1U;
  g_k4_next_disable = 1U;
}

static void ImuLevel_Reset(void)
{
  g_imu_level_active = 0U;
  g_imu_zero_pending = 0U;
  g_imu_zero_count = 0U;
  g_imu_zero_acc_sum = 0L;
  g_imu_acc_zero_mg = 0;
  g_imu_level_integral_mg = 0L;
  g_imu_level_last_error_mg = 0;
  g_ms901m_level_zero_cd = 0L;
  g_ms901m_level_zero_sum_cd = 0L;
  g_ms901m_level_error_cd = 0L;
  g_ms901m_level_zero_valid = 0U;
  g_accel_ff_zero_mg = 0;
  g_accel_ff_zero_sum = 0L;
  g_accel_ff_zero_count = 0U;
  g_accel_ff_zero_valid = 0U;
  g_accel_ff_steps = 0L;
  g_ms901m_axis_acc_mg = 0;
  g_ms901m_axis_acc_filtered_mg = 0;
  g_ms901m_acc_error_mg = 0;
  g_ms901m_acc_ok = 0U;
  g_ms901m_acc_filter_valid = 0U;
  g_ms901m_acc_jump_count = 0U;
}

static void ImuLevel_RequestHome(void)
{
  /* K1新流程：电机先回零，随后用当前MS901M轴向加速度作零点，进入中心稳定。 */
  Tracking_ResetPid();
  ImuLevel_Reset();
  g_tracking_enabled = 0U;
  g_swing_test_enabled = 0U;
  g_swing_dir = 0xFFU;
  g_balance_dir = 0xFFU;
  g_ball_task_state = BALL_TASK_IDLE;
  g_ball_task_arrive_ms = 0U;
  g_ball_target_mm = BALL_TARGET_DEFAULT_MM;
  g_manual_target_adjust_enabled = 1U;
  g_ball_filter_valid = 0U;
  BallLevel_Reset();
  K3Cascade_Reset(&g_k3_cascade);
  K3Cascade_Reset(&g_k2_cascade);
  g_after_home_mode = HOME_AFTER_IMU_LEVEL;
  g_balance_start_after_home_pending = 1U;
  g_motor_home_request = 1U;
}

static void ImuLevel_StartZeroAfterHome(void)
{
  Tracking_ResetPid();
  ImuLevel_Reset();
  g_ball_task_state = BALL_TASK_IDLE;
  g_ball_task_arrive_ms = 0U;
  g_motor_pos_steps = 0L;
  g_motor_target_steps = 0L;
  g_motor_disable_pending = 0U;
  g_motor_disable_last_tx_tick = 0U;
  Motor_Enable(1U);
  (void)Motor_SendSerialEnable(1U);
  Motor_ConfigStepGpio();
  Motor_Stop();
  g_tracking_enabled = 1U;
  g_imu_zero_pending = 1U;
  /* 保留K1回零期间由K5/K6选定的稳定位置。 */
  g_ball_filter_valid = 0U;
  BallLevel_Reset();
}

static uint8_t ImuLevel_Process20ms(void)
{
  int32_t control_steps;
  int32_t delta_steps;
#if (APP_MS901M_ENABLE == 0U)
  int32_t error_mg;
  int32_t derivative_mg;
#endif

  if ((g_imu_zero_pending == 0U) && (g_imu_level_active == 0U))
  {
    return 0U;
  }

#if (APP_MS901M_ENABLE != 0U)
  App_UpdateMs901mAccelSnapshot();
  if ((IMU901_HasFreshAccel(MS901M_Y_TIMEOUT_MS) == 0U) ||
      (IMU901_HasFreshAngle(MS901M_Y_TIMEOUT_MS) == 0U))
  {
    g_motor_target_steps = 0L;
    g_imu_level_integral_mg = 0L;
    g_imu_level_last_error_mg = 0;
    g_ms901m_level_zero_valid = 0U;
    return 1U;
  }

  if (g_imu_zero_pending != 0U)
  {
    g_motor_target_steps = 0L;
    if ((g_ms901m_acc_ok != 0U) && (g_ms901m_acc_filter_valid != 0U))
    {
      int32_t current_angle_cd = App_GetMs901mLevelAxisCd();

      if (g_imu_zero_count == 0U)
      {
        g_ms901m_level_zero_cd = current_angle_cd;
        g_ms901m_level_zero_sum_cd = 0L;
      }
      g_ms901m_level_zero_sum_cd +=
        App_Ms901mAngleDiffCd(current_angle_cd, g_ms901m_level_zero_cd);
      g_accel_ff_zero_sum += (int32_t)g_ms901m_axis_acc_filtered_mg;
      g_accel_ff_zero_count++;
      g_imu_zero_count++;
    }
    if (g_imu_zero_count >= IMU_LEVEL_ZERO_SAMPLES)
    {
      if (g_accel_ff_zero_count >= (IMU_LEVEL_ZERO_SAMPLES / 2U))
      {
        g_accel_ff_zero_mg = (int16_t)(g_accel_ff_zero_sum / (int32_t)g_accel_ff_zero_count);
        g_accel_ff_zero_valid = 1U;
      }
      else
      {
        g_accel_ff_zero_mg = g_ms901m_axis_acc_filtered_mg;
        g_accel_ff_zero_valid = 1U;
      }
      g_imu_acc_zero_mg = g_accel_ff_zero_mg;
      g_ms901m_level_zero_cd +=
        g_ms901m_level_zero_sum_cd / (int32_t)g_imu_zero_count;
      g_ms901m_level_error_cd = 0L;
      g_ms901m_level_zero_valid = 1U;
      g_imu_zero_pending = 0U;
      g_imu_level_active = 1U;
      g_imu_level_integral_mg = 0L;
      g_imu_level_last_error_mg = 0;
    }
    return 1U;
  }

  /* K1外环先给期望管角，随后由Roll内环把实际水管角拉到该目标。 */
  g_imu_level_integral_mg = 0L;
  g_imu_level_last_error_mg = 0;
  control_steps = 0L;
#else
  App_UpdateIcmSnapshot();
  if ((g_icm_ok == 0U) || (g_icm_acc_filter_valid == 0U))
  {
    g_motor_target_steps = 0L;
    g_imu_level_integral_mg = 0L;
    g_imu_level_last_error_mg = 0;
    return 1U;
  }

  if (g_imu_zero_pending != 0U)
  {
    g_motor_target_steps = 0L;
    g_imu_zero_acc_sum += (int32_t)g_icm_axis_acc_filtered_mg;
    g_imu_zero_count++;
    if (g_imu_zero_count >= IMU_LEVEL_ZERO_SAMPLES)
    {
      g_imu_acc_zero_mg = (int16_t)(g_imu_zero_acc_sum / (int32_t)g_imu_zero_count);
      g_imu_zero_pending = 0U;
      g_imu_level_active = 1U;
      g_imu_level_integral_mg = 0L;
      g_imu_level_last_error_mg = 0;
    }
    return 1U;
  }

  error_mg = (int32_t)g_icm_axis_acc_filtered_mg - (int32_t)g_imu_acc_zero_mg;
  if ((error_mg > -IMU_LEVEL_DEADBAND_MG) && (error_mg < IMU_LEVEL_DEADBAND_MG))
  {
    error_mg = 0L;
  }

  /* 车跑起来时MS901M只做加速度前馈，视觉继续负责把球锁在中心。 */
  g_imu_level_integral_mg += error_mg;
  if (g_imu_level_integral_mg > IMU_LEVEL_I_LIMIT_MG)
  {
    g_imu_level_integral_mg = IMU_LEVEL_I_LIMIT_MG;
  }
  else if (g_imu_level_integral_mg < -IMU_LEVEL_I_LIMIT_MG)
  {
    g_imu_level_integral_mg = -IMU_LEVEL_I_LIMIT_MG;
  }

  derivative_mg = error_mg - (int32_t)g_imu_level_last_error_mg;
  g_imu_level_last_error_mg = (int16_t)error_mg;

  control_steps = (IMU_LEVEL_SIGN * error_mg * (int32_t)MOTOR_STEPS_PER_REV) /
                  ACC_FF_SMALL_ANGLE_DEN;
  control_steps += (IMU_LEVEL_SIGN * g_imu_level_integral_mg) / IMU_LEVEL_KI_DEN;
  control_steps += (IMU_LEVEL_SIGN * derivative_mg * IMU_LEVEL_KD_NUM) / IMU_LEVEL_KD_DEN;
#endif

#if (APP_MS901M_ENABLE != 0U)
  control_steps = App_AddAccelFeedforward(0L);
  if (AppK230_HasFreshTarget(K230_TARGET_TIMEOUT_MS) != 0U)
  {
    /* K1和K3共用已经调好的球位置/球速度串级PID与提前刹车轨迹。 */
    control_steps += K3Cascade_Process20ms(&g_k3_cascade,
                                            (int32_t)AppK230_GetPosMm(),
                                            g_ball_target_mm);
  }
  else
  {
    K3Cascade_Reset(&g_k3_cascade);
  }
  control_steps = ImuLevel_ApplyMs901mAngleFeedback(control_steps);
#else
  control_steps = App_AddAccelFeedforward(control_steps);
  control_steps += BallLevel_Process20ms();
#endif
  if ((control_steps > -IMU_LEVEL_TARGET_DEADBAND_STEPS) &&
      (control_steps < IMU_LEVEL_TARGET_DEADBAND_STEPS))
  {
    control_steps = 0L;
  }

  if (control_steps > MOTOR_HARD_LIMIT_STEPS)
  {
    control_steps = MOTOR_HARD_LIMIT_STEPS;
  }
  else if (control_steps < -MOTOR_HARD_LIMIT_STEPS)
  {
    control_steps = -MOTOR_HARD_LIMIT_STEPS;
  }

  delta_steps = control_steps - g_motor_target_steps;
  if (delta_steps > IMU_LEVEL_SLEW_STEPS)
  {
    control_steps = g_motor_target_steps + IMU_LEVEL_SLEW_STEPS;
  }
  else if (delta_steps < -IMU_LEVEL_SLEW_STEPS)
  {
    control_steps = g_motor_target_steps - IMU_LEVEL_SLEW_STEPS;
  }
  g_motor_target_steps = control_steps;
  return 1U;
}

static uint8_t BallTask_Update20ms(int32_t abs_error, int32_t abs_velocity)
{
  if ((g_ball_task_state != BALL_TASK_GO_POS) && (g_ball_task_state != BALL_TASK_GO_NEG))
  {
    return 0U;
  }

  if (g_ball_task_state == BALL_TASK_GO_POS)
  {
    if (abs_error <= 10L)
    {
      g_ball_task_state = BALL_TASK_GO_NEG;
      Ball_SetTarget(g_k2_center_mm + BALL_TASK_NEG_TARGET_MM);
      return 1U;
    }
    return 0U;
  }

  if (g_ball_task_state == BALL_TASK_GO_NEG)
  {
    if ((abs_error <= BALL_TASK_ARRIVE_ZONE_MM) && (abs_velocity <= BALL_TASK_ARRIVE_SPEED_MM))
    {
      if (g_ball_task_arrive_ms < BALL_TASK_ARRIVE_HOLD_MS)
      {
        g_ball_task_arrive_ms += 20U;
      }
    }
    else
    {
      g_ball_task_arrive_ms = 0U;
      return 0U;
    }

    if (g_ball_task_arrive_ms < BALL_TASK_ARRIVE_HOLD_MS)
    {
      return 0U;
    }

    g_ball_task_arrive_ms = 0U;
    g_ball_task_state = BALL_TASK_HOLD_NEG;
    return 0U;
  }

  return 0U;
}

static void BallTask_CascadeControl20ms(int32_t raw_pos_mm)
{
  int32_t error_q8;
  int32_t abs_error_q8;
  int32_t abs_velocity_q8;
  int32_t abs_error_mm;
  int32_t abs_velocity_mm;

  /* Timeout removed. Will hold at -5cm indefinitely. */

  g_motor_target_steps = K3Cascade_Process20ms(&g_k2_cascade, raw_pos_mm,
                                                 g_ball_target_mm);

  g_motor_target_steps += (g_pa8_ff_val + g_pa11_ff_val);

  if (g_ball_task_state == BALL_TASK_GO_POS)
  {
    g_motor_target_steps += -15L;
  }
  else if ((g_ball_task_state == BALL_TASK_GO_NEG) || (g_ball_task_state == BALL_TASK_HOLD_NEG))
  {
    g_motor_target_steps += -20L;
  }

  /* 璇樊涓庡垽瀹氱姸鎬?*/
  error_q8 = (g_ball_target_mm * K3_CASCADE_Q8) - g_k2_cascade.pos_q8;
  abs_error_q8 = (error_q8 >= 0L) ? error_q8 : -error_q8;
  abs_velocity_q8 = (g_k2_cascade.velocity_q8 >= 0L) ?
                    g_k2_cascade.velocity_q8 : -g_k2_cascade.velocity_q8;
  abs_error_mm = (abs_error_q8 + K3_CASCADE_Q8 - 1L) / K3_CASCADE_Q8;
  abs_velocity_mm = (abs_velocity_q8 + K3_CASCADE_Q8 - 1L) / K3_CASCADE_Q8;

  (void)BallTask_Update20ms(abs_error_mm, abs_velocity_mm);
}

static uint8_t BallTask_IsSequenceState(void)
{
  return ((g_ball_task_state == BALL_TASK_GO_POS) ||
          (g_ball_task_state == BALL_TASK_GO_NEG) ||
          (g_ball_task_state == BALL_TASK_HOLD_NEG)) ? 1U : 0U;
}

static int32_t BallTask_GetSettleBiasSteps(void)
{
  if (BallTask_IsSequenceState() == 0U)
  {
    return 0L;
  }

  return BallBalance_StaticSteps(g_ball_target_mm, (int32_t)MOTOR_STEPS_PER_REV);
}

static void BallTask_Control20ms(int16_t error, int32_t abs_error, int32_t velocity,
                                 int32_t abs_velocity, uint8_t moving_toward_target,
                                 uint8_t predicted_cross_target)
{
  int32_t control;
  int32_t derivative;
  int32_t abs_control;
  int32_t drive_sign;
  int32_t brake_steps;
  int32_t brake_line_mm;
  int32_t brake_distance_error;
  int32_t delta_steps;
  int32_t slew_steps;
  uint8_t drive_ticks;
  uint8_t flat_ticks;
  uint8_t static_kick_now;
  uint8_t crossed_target;

  if (g_ball_task_quiet_latched != 0U)
  {
    if (abs_error <= BALL_TASK_QUIET_RELEASE_MM)
    {
      g_ball_task_pid_integral = 0L;
      g_ball_task_pid_last_error = error;
      g_ball_pulse_ticks = 0U;
      g_ball_pulse_coast_ticks = 0U;
      g_ball_pulse_coast_reload = 0U;
      g_ball_pulse_control_steps = 0L;
      g_ball_task_static_recall_ticks = 0U;
      g_ball_task_static_recall_steps = 0L;
      g_motor_target_steps = BallTask_GetSettleBiasSteps();
      return;
    }
    g_ball_task_quiet_latched = 0U;
  }

  if (((BallTask_IsSequenceState() == 0U) && (abs_error <= BALL_TASK_QUIET_ZONE_MM) &&
       (abs_velocity <= BALL_TASK_QUIET_SPEED_MM)) ||
      ((BallTask_IsSequenceState() != 0U) && (abs_error <= BALL_TASK_ARRIVE_ZONE_MM) &&
       (abs_velocity <= BALL_TASK_QUIET_SPEED_MM)))
  {
    g_ball_task_pid_integral = 0L;
    g_ball_task_pid_last_error = error;
    Ball_ResetPulseControl();
    g_ball_task_quiet_latched = 1U;
    g_motor_target_steps = BallTask_GetSettleBiasSteps();
    return;
  }

  static_kick_now = 0U;
  g_ball_task_pid_integral += error;
  if (g_ball_task_pid_integral > BALL_TASK_PID_I_LIMIT)
  {
    g_ball_task_pid_integral = BALL_TASK_PID_I_LIMIT;
  }
  if (g_ball_task_pid_integral < -BALL_TASK_PID_I_LIMIT)
  {
    g_ball_task_pid_integral = -BALL_TASK_PID_I_LIMIT;
  }

  derivative = (int32_t)error - (int32_t)g_ball_task_pid_last_error;
  g_ball_task_pid_last_error = error;
  drive_sign = (BALL_CONTROL_SIGN * (int32_t)error >= 0L) ? 1L : -1L;
  crossed_target =
    (((g_ball_task_state == BALL_TASK_GO_POS) && (error < 0)) ||
     (((g_ball_task_state == BALL_TASK_GO_NEG) ||
       (g_ball_task_state == BALL_TASK_HOLD_NEG)) && (error > 0))) ? 1U : 0U;

  control = BALL_CONTROL_SIGN *
            (((BALL_TASK_PID_KP_NUM * (int32_t)error) +
              (BALL_TASK_PID_KI_NUM * g_ball_task_pid_integral) +
              (BALL_TASK_PID_KD_NUM * derivative)) / BALL_TASK_PID_DEN);
  if (control == 0L)
  {
    control = BALL_CONTROL_SIGN * (int32_t)error;
  }

  abs_control = (control >= 0L) ? control : -control;
  if (abs_error <= BALL_TASK_MICRO_ZONE_MM)
  {
    if (abs_control < BALL_TASK_NEAR_MIN_STEPS)
    {
      abs_control = BALL_TASK_NEAR_MIN_STEPS;
    }
    if (abs_control > BALL_TASK_NEAR_MAX_STEPS)
    {
      abs_control = BALL_TASK_NEAR_MAX_STEPS;
    }
  }
  else if (abs_error <= BALL_TASK_MID_ZONE_MM)
  {
    if (abs_control < BALL_TASK_MID_MIN_STEPS)
    {
      abs_control = BALL_TASK_MID_MIN_STEPS;
    }
    if (abs_control > BALL_TASK_MID_MAX2_STEPS)
    {
      abs_control = BALL_TASK_MID_MAX2_STEPS;
    }
  }
  else
  {
    if (abs_control < BALL_TASK_KICK_MIN_STEPS)
    {
      abs_control = BALL_TASK_KICK_MIN_STEPS;
    }
    if ((abs_error > BALL_TASK_STATIC_KICK_ZONE_MM) &&
        !(((g_ball_task_state == BALL_TASK_GO_NEG) ||
           (g_ball_task_state == BALL_TASK_HOLD_NEG)) &&
          (abs_error <= BALL_TASK_NEG_NO_LONG_KICK_MM)) &&
        (abs_velocity <= BALL_HOLD_SPEED_MM) &&
        (abs_control < BALL_TASK_STATIC_KICK_STEPS))
    {
      abs_control = BALL_TASK_STATIC_KICK_STEPS;
      static_kick_now = 1U;
    }
    if (abs_control > BALL_TASK_KICK_MAX_STEPS)
    {
      abs_control = BALL_TASK_KICK_MAX_STEPS;
    }
    if ((abs_error <= BALL_MID_ZONE_MM) && (abs_velocity > BALL_HOLD_SPEED_MM) &&
        (abs_control > BALL_TASK_MID_MAX_STEPS))
    {
      abs_control = BALL_TASK_MID_MAX_STEPS;
    }
  }

  brake_line_mm = BALL_TASK_BRAKE_BASE_MM + (abs_velocity * BALL_TASK_BRAKE_VEL_GAIN);
  if (brake_line_mm > BALL_TASK_BRAKE_MAX_MM)
  {
    brake_line_mm = BALL_TASK_BRAKE_MAX_MM;
  }
  if (abs_error > BALL_PID_ZONE_MM)
  {
    brake_line_mm -= BALL_TASK_OUTER_DELAY_MM;
    if (brake_line_mm < BALL_PID_ZONE_MM)
    {
      brake_line_mm = BALL_PID_ZONE_MM;
    }
  }

  if ((g_ball_task_state == BALL_TASK_GO_POS) ||
      (g_ball_task_state == BALL_TASK_GO_NEG) ||
      (g_ball_task_state == BALL_TASK_HOLD_NEG))
  {
    /* 赛题3全程复用中心稳定的动态刹车逻辑。 */
    if ((crossed_target != 0U) && (abs_error <= BALL_TASK_SETTLE_ZONE_MM))
    {
      /* 到正负5cm的1cm合格区内先回平等停，避免-5.1cm马上反打回-4cm。 */
      g_ball_task_pid_integral = 0L;
      g_ball_task_pid_last_error = error;
      Ball_ResetPulseControl();
      g_motor_target_steps = BallTask_GetSettleBiasSteps();
      if (abs_velocity <= BALL_TASK_QUIET_SPEED_MM)
      {
        g_ball_task_quiet_latched = 1U;
      }
      return;
    }

    if (((g_ball_task_state == BALL_TASK_GO_NEG) ||
         (g_ball_task_state == BALL_TASK_HOLD_NEG)) &&
        (abs_error <= BALL_TASK_NEG_FINE_ZONE_MM))
    {
      /* 靠近-5cm后绝不延续远处启动的大角度保持，避免继续压过目标。 */
      g_ball_task_static_kick_ticks = 0U;
      g_ball_task_static_kick_steps = 0L;
      g_ball_task_static_recall_ticks = 0U;
      g_ball_task_static_recall_steps = 0L;
    }

    if (g_ball_task_static_recall_ticks > 0U)
    {
      g_ball_task_static_recall_ticks--;
      control = g_ball_task_static_recall_steps;
      goto apply_task_control;
    }
    if (g_ball_task_static_kick_ticks > 0U)
    {
      if ((moving_toward_target != 0U) && (abs_velocity >= BALL_TASK_START_SPEED_MM))
      {
        g_ball_task_static_kick_ticks = 0U;
        control = (g_ball_task_static_kick_steps >= 0L) ?
                  -BALL_TASK_STATIC_RECALL_STEPS : BALL_TASK_STATIC_RECALL_STEPS;
      }
      else
      {
        g_ball_task_static_kick_ticks--;
        control = g_ball_task_static_kick_steps;
        if (g_ball_task_static_kick_ticks == 0U)
        {
          g_ball_task_static_recall_ticks = 1U;
          g_ball_task_static_recall_steps =
            (g_ball_task_static_kick_steps >= 0L) ?
            -BALL_TASK_STATIC_RECALL_STEPS : BALL_TASK_STATIC_RECALL_STEPS;
        }
      }
      goto apply_task_control;
    }

    if ((g_ball_task_state == BALL_TASK_GO_NEG) || (g_ball_task_state == BALL_TASK_HOLD_NEG))
    {
      brake_line_mm = BALL_TASK_NEG_BRAKE_BASE_MM +
                      (abs_velocity * BALL_TASK_NEG_BRAKE_VEL_GAIN);
    }
    else
    {
      brake_line_mm = BALL_BRAKE_BASE_MM + (abs_velocity * BALL_BRAKE_VEL_GAIN);
    }
    if (brake_line_mm > BALL_BRAKE_DIST_MAX_MM)
    {
      brake_line_mm = BALL_BRAKE_DIST_MAX_MM;
    }

    if (moving_toward_target != 0U)
    {
      if (predicted_cross_target == 0U)
      {
        control = 0L;
      }
      else
      {
        brake_steps = abs_velocity * BALL_PID_KD_NUM / BALL_PID_DEN;
        brake_distance_error = brake_line_mm - abs_error;
        if (brake_distance_error > 0L)
        {
          brake_steps += brake_distance_error * BALL_BRAKE_DIST_GAIN;
        }
        if ((abs_error <= BALL_SLOW_ZONE_MM) && (brake_steps < BALL_NEAR_BRAKE_MIN_STEPS))
        {
          brake_steps = BALL_NEAR_BRAKE_MIN_STEPS;
        }
        else if (brake_steps < BALL_BRAKE_MIN_STEPS)
        {
          brake_steps = BALL_BRAKE_MIN_STEPS;
        }
        if ((abs_error <= BALL_SLOW_ZONE_MM) && (brake_steps > BALL_NEAR_BRAKE_MAX_STEPS))
        {
          brake_steps = BALL_NEAR_BRAKE_MAX_STEPS;
        }
        if (brake_steps > BALL_BRAKE_MAX_STEPS)
        {
          brake_steps = BALL_BRAKE_MAX_STEPS;
        }
        if (((g_ball_task_state == BALL_TASK_GO_NEG) ||
             (g_ball_task_state == BALL_TASK_HOLD_NEG)) &&
            (abs_error <= BALL_TASK_NEG_FINE_ZONE_MM) &&
            (brake_steps > BALL_TASK_NEG_FINE_BRAKE_MAX))
        {
          brake_steps = BALL_TASK_NEG_FINE_BRAKE_MAX;
        }
        control = -drive_sign * brake_steps;
      }
    }
    else
    {
      if (((g_ball_task_state == BALL_TASK_GO_NEG) ||
           (g_ball_task_state == BALL_TASK_HOLD_NEG)) &&
          (abs_error <= BALL_TASK_NEG_FINE_ZONE_MM))
      {
        control = drive_sign *
                  (((abs_error > BALL_TASK_SETTLE_ZONE_MM) &&
                    (abs_velocity <= BALL_TASK_REKICK_SPEED_MM)) ?
                   BALL_TASK_NEAR_STALL_KICK_STEPS : BALL_TASK_NEG_FINE_STEPS);
        goto apply_task_control;
      }

      control = drive_sign * abs_control;
      if ((abs_error > BALL_TASK_STATIC_KICK_ZONE_MM) &&
          !(((g_ball_task_state == BALL_TASK_GO_NEG) ||
             (g_ball_task_state == BALL_TASK_HOLD_NEG)) &&
            (abs_error <= BALL_TASK_NEG_NO_LONG_KICK_MM)) &&
          (abs_velocity <= BALL_HOLD_SPEED_MM) &&
          (control > -BALL_TASK_STATIC_KICK_STEPS) && (control < BALL_TASK_STATIC_KICK_STEPS))
      {
        control = drive_sign *
                  ((abs_error <= BALL_TASK_MICRO_ZONE_MM) ?
                   BALL_TASK_NEAR_STALL_KICK_STEPS : BALL_TASK_STATIC_KICK_STEPS);
        g_ball_task_static_kick_steps = control;
        if (abs_error <= BALL_TASK_MICRO_ZONE_MM)
        {
          g_ball_task_static_kick_ticks = BALL_TASK_STATIC_KICK_NEAR_TICKS - 1U;
        }
        else if (abs_error <= BALL_TASK_MID_ZONE_MM)
        {
          g_ball_task_static_kick_ticks = BALL_TASK_STATIC_KICK_MID_TICKS - 1U;
        }
        else
        {
          g_ball_task_static_kick_ticks = BALL_TASK_STATIC_KICK_FAR_TICKS - 1U;
        }
      }
    }
    goto apply_task_control;
  }

  /* 赛题3独立控制：DRIVE->FLAT->BRAKE->FLAT，禁止每帧正反抖动。 */
  if (g_ball_task_stage != BALL_TASK_STAGE_IDLE)
  {
    if ((g_ball_task_stage == BALL_TASK_STAGE_DRIVE) ||
        (g_ball_task_stage == BALL_TASK_STAGE_BRAKE))
    {
      control = g_ball_task_stage_steps;
    }
    else
    {
      control = 0L;
    }

    if ((g_ball_task_stage == BALL_TASK_STAGE_DRIVE) &&
        (moving_toward_target != 0U) && (abs_velocity >= BALL_TASK_START_SPEED_MM))
    {
      g_ball_task_stage = BALL_TASK_STAGE_FLAT;
      g_ball_task_stage_ticks = g_ball_task_flat_reload_ticks;
      g_ball_task_stage_steps = 0L;
      control = 0L;
      goto apply_task_control;
    }

    if (g_ball_task_stage_ticks > 0U)
    {
      g_ball_task_stage_ticks--;
      goto apply_task_control;
    }

    if ((g_ball_task_stage == BALL_TASK_STAGE_DRIVE) ||
        (g_ball_task_stage == BALL_TASK_STAGE_BRAKE))
    {
      g_ball_task_stage = (g_ball_task_stage == BALL_TASK_STAGE_BRAKE) ?
                          BALL_TASK_STAGE_BRAKE_DONE : BALL_TASK_STAGE_FLAT;
      g_ball_task_stage_ticks = g_ball_task_flat_reload_ticks;
      g_ball_task_stage_steps = 0L;
      control = 0L;
      goto apply_task_control;
    }

    if ((g_ball_task_stage == BALL_TASK_STAGE_FLAT) &&
        (moving_toward_target != 0U) && (abs_velocity > BALL_TASK_COAST_MIN_SPEED_MM) &&
        ((abs_error <= brake_line_mm) || (predicted_cross_target != 0U)))
    {
      brake_steps = abs_velocity * BALL_TASK_PID_KD_NUM / BALL_TASK_PID_DEN;
      brake_distance_error = brake_line_mm - abs_error;
      if (brake_distance_error > 0L)
      {
        brake_steps += brake_distance_error * BALL_TASK_BRAKE_DIST_GAIN;
      }
      if ((abs_error <= BALL_SLOW_ZONE_MM) && (brake_steps < BALL_TASK_NEAR_BRAKE_MIN))
      {
        brake_steps = BALL_TASK_NEAR_BRAKE_MIN;
      }
      else if (brake_steps < BALL_TASK_BRAKE_MIN_STEPS)
      {
        brake_steps = BALL_TASK_BRAKE_MIN_STEPS;
      }
      if ((abs_error <= BALL_SLOW_ZONE_MM) && (brake_steps > BALL_TASK_NEAR_BRAKE_MAX))
      {
        brake_steps = BALL_TASK_NEAR_BRAKE_MAX;
      }
      if (brake_steps > BALL_TASK_BRAKE_MAX_STEPS)
      {
        brake_steps = BALL_TASK_BRAKE_MAX_STEPS;
      }

      control = -drive_sign * brake_steps;
      g_ball_task_stage = BALL_TASK_STAGE_BRAKE;
      g_ball_task_stage_ticks = BALL_TASK_MICRO_PULSE_TICKS - 1U;
      g_ball_task_flat_reload_ticks = BALL_TASK_BRAKE_COAST_TICKS;
      g_ball_task_stage_steps = control;
      goto apply_task_control;
    }

    if (((g_ball_task_stage == BALL_TASK_STAGE_FLAT) ||
         (g_ball_task_stage == BALL_TASK_STAGE_BRAKE_DONE)) &&
        (abs_error > BALL_TASK_QUIET_RELEASE_MM) &&
        ((abs_velocity <= BALL_TASK_REKICK_SPEED_MM) || (moving_toward_target == 0U)))
    {
      drive_ticks = (abs_error <= BALL_TASK_MICRO_ZONE_MM) ? BALL_TASK_MICRO_PULSE_TICKS :
                    ((abs_error <= BALL_TASK_MID_ZONE_MM) ? BALL_TASK_MID_PULSE_TICKS :
                     BALL_TASK_DRIVE_PULSE_TICKS);
      flat_ticks = (abs_error <= BALL_TASK_MICRO_ZONE_MM) ?
                   BALL_TASK_MICRO_COAST_TICKS : BALL_TASK_DRIVE_COAST_TICKS;
      control = drive_sign * abs_control;
      g_ball_task_flat_reload_ticks = flat_ticks;
      g_ball_task_stage_steps = control;
      if (drive_ticks > 1U)
      {
        g_ball_task_stage = BALL_TASK_STAGE_DRIVE;
        g_ball_task_stage_ticks = drive_ticks - 1U;
      }
      else
      {
        g_ball_task_stage = BALL_TASK_STAGE_FLAT;
        g_ball_task_stage_ticks = g_ball_task_flat_reload_ticks;
        g_ball_task_stage_steps = 0L;
      }
    }
    goto apply_task_control;
  }

  if ((moving_toward_target != 0U) && (abs_velocity > BALL_TASK_COAST_MIN_SPEED_MM))
  {
    if ((abs_error > brake_line_mm) && (predicted_cross_target == 0U))
    {
      control = 0L;
      goto apply_task_control;
    }

    brake_steps = abs_velocity * BALL_TASK_PID_KD_NUM / BALL_TASK_PID_DEN;
    brake_distance_error = brake_line_mm - abs_error;
    if (brake_distance_error > 0L)
    {
      brake_steps += brake_distance_error * BALL_TASK_BRAKE_DIST_GAIN;
    }
    if ((abs_error <= BALL_SLOW_ZONE_MM) && (brake_steps < BALL_TASK_NEAR_BRAKE_MIN))
    {
      brake_steps = BALL_TASK_NEAR_BRAKE_MIN;
    }
    else if (brake_steps < BALL_TASK_BRAKE_MIN_STEPS)
    {
      brake_steps = BALL_TASK_BRAKE_MIN_STEPS;
    }
    if ((abs_error <= BALL_SLOW_ZONE_MM) && (brake_steps > BALL_TASK_NEAR_BRAKE_MAX))
    {
      brake_steps = BALL_TASK_NEAR_BRAKE_MAX;
    }
    if (brake_steps > BALL_TASK_BRAKE_MAX_STEPS)
    {
      brake_steps = BALL_TASK_BRAKE_MAX_STEPS;
    }

    control = -drive_sign * brake_steps;
    g_ball_task_stage_steps = control;
    g_ball_task_flat_reload_ticks = BALL_TASK_BRAKE_COAST_TICKS;
    if (BALL_TASK_MICRO_PULSE_TICKS > 1U)
    {
      g_ball_task_stage = BALL_TASK_STAGE_BRAKE;
      g_ball_task_stage_ticks = BALL_TASK_MICRO_PULSE_TICKS - 1U;
    }
    else
    {
      g_ball_task_stage = BALL_TASK_STAGE_FLAT;
      g_ball_task_stage_ticks = BALL_TASK_BRAKE_COAST_TICKS;
      g_ball_task_stage_steps = 0L;
    }
  }
  else
  {
    drive_ticks = (abs_error <= BALL_TASK_MICRO_ZONE_MM) ? BALL_TASK_MICRO_PULSE_TICKS :
                  ((abs_error <= BALL_TASK_MID_ZONE_MM) ? BALL_TASK_MID_PULSE_TICKS :
                   BALL_TASK_DRIVE_PULSE_TICKS);
    flat_ticks = (abs_error <= BALL_TASK_MICRO_ZONE_MM) ?
                 BALL_TASK_MICRO_COAST_TICKS : BALL_TASK_DRIVE_COAST_TICKS;

    control = drive_sign * abs_control;
    g_ball_task_flat_reload_ticks = flat_ticks;
    if (static_kick_now != 0U)
    {
      g_ball_task_flat_reload_ticks = BALL_TASK_DRIVE_COAST_TICKS;
    }
    if (drive_ticks > 1U)
    {
      g_ball_task_stage = BALL_TASK_STAGE_DRIVE;
      g_ball_task_stage_ticks = drive_ticks - 1U;
      g_ball_task_stage_steps = control;
    }
    else
    {
      g_ball_task_stage = BALL_TASK_STAGE_FLAT;
      g_ball_task_stage_ticks = g_ball_task_flat_reload_ticks;
      g_ball_task_stage_steps = 0L;
    }
  }

apply_task_control:
  control = App_AddAccelFeedforward(control);
  if (control > MOTOR_HARD_LIMIT_STEPS)
  {
    control = MOTOR_HARD_LIMIT_STEPS;
  }
  else if (control < -MOTOR_HARD_LIMIT_STEPS)
  {
    control = -MOTOR_HARD_LIMIT_STEPS;
  }

  delta_steps = control - g_motor_target_steps;
  slew_steps = (((control > 0L) && (g_motor_target_steps < 0L)) ||
                ((control < 0L) && (g_motor_target_steps > 0L))) ?
               BALL_TASK_BRAKE_SLEW_STEPS : BALL_TASK_SLEW_STEPS;
  if (delta_steps > slew_steps)
  {
    control = g_motor_target_steps + slew_steps;
  }
  else if (delta_steps < -slew_steps)
  {
    control = g_motor_target_steps - slew_steps;
  }

  g_motor_target_steps = control;
}

static void Tracking_StartAtTarget(int32_t target_mm)
{
  if (g_tracking_enabled != 0U)
  {
    return;
  }

  g_motor_output_locked = 0U;
  g_motor_disable_request = 0U;
  g_motor_disable_pending = 0U;
  g_motor_disable_last_tx_tick = 0U;
  Motor_Enable(1U);
  (void)Motor_SendSerialEnable(1U);
  Tracking_ResetPid();
  Ball_SetTarget(target_mm);
  g_tracking_enabled = 1U;
  g_swing_test_enabled = 0U;
  g_swing_target_sign = 1;
  g_swing_dwell_ticks = 0U;
  g_swing_dir = 0xFFU;
  g_balance_dir = 0xFFU;
  g_motor_target_steps = g_motor_pos_steps;
  Motor_ConfigStepGpio();
  Motor_Stop();
}

static void SwingTest_Process20ms(void)
{
  uint8_t dir;

  if ((g_swing_test_enabled == 0U) || (g_motor_enabled == 0U))
  {
    return;
  }

  if (g_swing_dwell_ticks > 0U)
  {
    g_swing_dwell_ticks--;
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    g_swing_step_high = 0U;
    g_swing_step_wait_ms = 0U;
    return;
  }

  if ((g_swing_target_sign > 0) && (g_motor_pos_steps >= MOTOR_HARD_LIMIT_STEPS))
  {
    g_swing_target_sign = -1;
    g_swing_dwell_ticks = MOTOR_SWING_DWELL_TICKS;
    g_swing_dir = 0xFFU;
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    g_swing_step_high = 0U;
    g_swing_step_wait_ms = 0U;
    return;
  }

  if ((g_swing_target_sign < 0) && (g_motor_pos_steps <= -MOTOR_HARD_LIMIT_STEPS))
  {
    g_swing_target_sign = 1;
    g_swing_dwell_ticks = MOTOR_SWING_DWELL_TICKS;
    g_swing_dir = 0xFFU;
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    g_swing_step_high = 0U;
    g_swing_step_wait_ms = 0U;
    return;
  }

  dir = (g_swing_target_sign > 0) ? MOTOR_POSITIVE_DIR : (uint8_t)(1U - MOTOR_POSITIVE_DIR);
  if (dir != g_swing_dir)
  {
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, (dir == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    g_swing_dir = dir;
    g_swing_step_high = 0U;
    g_swing_step_wait_ms = 1U; /* 换向后等1ms，保证DIR建立时间。 */
  }
}

static void SwingTest_Process1ms(void)
{
  int32_t next_pos;

  if ((g_swing_test_enabled == 0U) || (g_motor_enabled == 0U) || (g_swing_dwell_ticks > 0U))
  {
    return;
  }

  if (g_swing_step_high != 0U)
  {
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    g_swing_step_high = 0U;
    return;
  }

  if (g_swing_step_wait_ms > 0U)
  {
    g_swing_step_wait_ms--;
    return;
  }

  if ((g_swing_target_sign > 0) && (g_motor_pos_steps >= MOTOR_HARD_LIMIT_STEPS))
  {
    return;
  }
  if ((g_swing_target_sign < 0) && (g_motor_pos_steps <= -MOTOR_HARD_LIMIT_STEPS))
  {
    return;
  }

  next_pos = g_motor_pos_steps + (int32_t)g_swing_target_sign;
  if (next_pos > MOTOR_HARD_LIMIT_STEPS)
  {
    next_pos = MOTOR_HARD_LIMIT_STEPS;
  }
  else if (next_pos < -MOTOR_HARD_LIMIT_STEPS)
  {
    next_pos = -MOTOR_HARD_LIMIT_STEPS;
  }

  HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_SET);
  g_swing_step_high = 1U;
  g_swing_step_wait_ms = (MOTOR_SWING_STEP_INTERVAL_MS > 2U) ? (MOTOR_SWING_STEP_INTERVAL_MS - 2U) : 0U;
  g_motor_pos_steps = next_pos;
}

static void BalanceMotor_Process1ms(void)
{
  int32_t diff;
  int32_t next_pos;
  uint8_t dir;
  uint8_t interval;

  if ((g_tracking_enabled == 0U) || (g_swing_test_enabled != 0U) || (g_motor_enabled == 0U))
  {
    return;
  }

  if (g_balance_step_high != 0U)
  {
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    g_balance_step_high = 0U;
    return;
  }

  if (g_balance_step_wait_ms > 0U)
  {
    g_balance_step_wait_ms--;
    return;
  }

  if (g_motor_target_steps > MOTOR_HARD_LIMIT_STEPS)
  {
    g_motor_target_steps = MOTOR_HARD_LIMIT_STEPS;
  }
  else if (g_motor_target_steps < -MOTOR_HARD_LIMIT_STEPS)
  {
    g_motor_target_steps = -MOTOR_HARD_LIMIT_STEPS;
  }

  diff = g_motor_target_steps - g_motor_pos_steps;
  if (diff == 0L)
  {
    g_balance_dir = 0xFFU;
    return;
  }

  if (((diff > 0L) && (g_motor_pos_steps >= MOTOR_HARD_LIMIT_STEPS)) ||
      ((diff < 0L) && (g_motor_pos_steps <= -MOTOR_HARD_LIMIT_STEPS)))
  {
    g_motor_target_steps = g_motor_pos_steps;
    g_balance_dir = 0xFFU;
    return;
  }

  dir = (diff > 0L) ? MOTOR_POSITIVE_DIR : (uint8_t)(1U - MOTOR_POSITIVE_DIR);
  if (dir != g_balance_dir)
  {
    HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(X_DIR_GPIO_Port, X_DIR_Pin, (dir == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    g_balance_dir = dir;
    g_balance_step_wait_ms = 1U; /* 换向后等1ms，保证DIR建立时间。 */
    return;
  }

  next_pos = g_motor_pos_steps + ((diff > 0L) ? 1L : -1L);
  if (next_pos > MOTOR_HARD_LIMIT_STEPS)
  {
    next_pos = MOTOR_HARD_LIMIT_STEPS;
  }
  else if (next_pos < -MOTOR_HARD_LIMIT_STEPS)
  {
    next_pos = -MOTOR_HARD_LIMIT_STEPS;
  }

  HAL_GPIO_WritePin(MOTOR_STEP_GPIO_Port, MOTOR_STEP_Pin, GPIO_PIN_SET);
  g_balance_step_high = 1U;
  interval = ((diff > -MOTOR_STEP_SLOW_BAND) && (diff < MOTOR_STEP_SLOW_BAND)) ?
             MOTOR_STEP_SLOW_INTERVAL_MS : MOTOR_STEP_FAST_INTERVAL_MS;
  g_balance_step_wait_ms = (interval > 2U) ? (interval - 2U) : 0U;
  g_motor_pos_steps = next_pos;
}

static void Tracking_Process20ms(void)
{
  int16_t error;
  int32_t control;
  int32_t derivative;
  int32_t abs_error;
  int32_t abs_control;
  int32_t drive_sign;
  int32_t brake_steps;
  int32_t raw_pos;
  int32_t velocity;
  int32_t abs_velocity;
  int32_t predicted_pos;
  int32_t predicted_error;
  int32_t delta_steps;
  int32_t slew_steps;
  int32_t dynamic_brake_distance;
  int32_t outer_brake_distance;
  int32_t brake_distance_error;
  uint8_t moving_toward_target;
  uint8_t predicted_cross_target;
  uint8_t start_pulse;
  uint8_t pulse_ticks;
  uint8_t coast_ticks;
  uint8_t vision_available;

  Motor_UpdateEstimatedPosition20ms();

  if (g_motor_output_locked != 0U)
  {
    Tracking_ResetPid();
    Motor_Stop();
    HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, MOTOR_EN_DISABLED_LEVEL);
    g_motor_enabled = 0U;
    return;
  }

  if ((g_tracking_enabled == 0U) || (g_motor_enabled == 0U))
  {
    Tracking_ResetPid();
    Motor_Stop();
    return;
  }

  if (ImuLevel_Process20ms() != 0U)
  {
    return;
  }

  if (g_swing_test_enabled != 0U)
  {
    Tracking_ResetPid();
    SwingTest_Process20ms();
    return;
  }

  if (g_ball_task_state == BALL_TASK_FINISHED)
  {
    /* K2赛题3已到4.5秒：电机回平，但不再执行任何视觉位置调节。 */
    g_motor_target_steps = 0L;
    return;
  }

  vision_available = AppK230_HasFreshTarget(K230_TARGET_TIMEOUT_MS);
  if (vision_available == 0U)
  {
    AppK230_Invalidate();
    if (BallTask_IsSequenceState() == 0U)
    {
      Tracking_ResetPid();
      g_motor_target_steps = g_motor_pos_steps;
      Motor_Stop();
      return;
    }
  }

  /* 第三问不能等识别到钢球才动：无视觉时用上一次位置，没有历史位置就按O点启动。 */
  raw_pos = (vision_available != 0U) ? (int32_t)AppK230_GetPosMm() :
            ((g_ball_filter_valid != 0U) ? g_ball_pos_filtered_mm : BALL_TARGET_DEFAULT_MM);

  if (BallTask_IsSequenceState() != 0U)
  {
    /* K2赛题3使用独立的K3串级PID，不再进入旧脉冲/补踢控制。 */
    BallTask_CascadeControl20ms(raw_pos);
    return;
  }

  if (g_ball_filter_valid == 0U)
  {
    g_ball_pos_filtered_mm = raw_pos;
    g_ball_pos_last_filtered_mm = raw_pos;
    g_ball_filter_valid = 1U;
  }
  else
  {
    g_ball_pos_last_filtered_mm = g_ball_pos_filtered_mm;
    g_ball_pos_filtered_mm =
      ((g_ball_pos_filtered_mm * BALL_FILTER_NUM) + raw_pos) / BALL_FILTER_DEN;
  }

  velocity = g_ball_pos_filtered_mm - g_ball_pos_last_filtered_mm;
  abs_velocity = (velocity >= 0L) ? velocity : -velocity;
  predicted_pos = g_ball_pos_filtered_mm + (velocity * BALL_PREDICT_CYCLES);
  error = (int16_t)(g_ball_target_mm - g_ball_pos_filtered_mm);
  predicted_error = g_ball_target_mm - predicted_pos;
  abs_error = (error >= 0) ? (int32_t)error : -(int32_t)error;
  moving_toward_target = (((int32_t)error * velocity) > 0L) ? 1U : 0U;
  predicted_cross_target = (((int32_t)error * predicted_error) <= 0L) ? 1U : 0U;
  dynamic_brake_distance = BALL_BRAKE_BASE_MM + (abs_velocity * BALL_BRAKE_VEL_GAIN);
  if (dynamic_brake_distance > BALL_BRAKE_DIST_MAX_MM)
  {
    dynamic_brake_distance = BALL_BRAKE_DIST_MAX_MM;
  }
  outer_brake_distance = dynamic_brake_distance - BALL_OUTER_BRAKE_DELAY_MM;
  if (outer_brake_distance < BALL_PID_ZONE_MM)
  {
    outer_brake_distance = BALL_PID_ZONE_MM;
  }

  if ((g_ball_task_state == BALL_TASK_HOLD_CENTER) || (g_ball_task_state == BALL_TASK_DEBUG_FF))
    {
      int32_t k3_mech_offset = 0L;
    
    if (g_ball_target_mm >= 30L)
      {
        k3_mech_offset = -20L - (((g_ball_target_mm - 30L) * (g_k3_offset_10cm - 20L)) / 70L);
      }
      else if (g_ball_target_mm <= -30L)
      {
        k3_mech_offset = 20L + (((-g_ball_target_mm - 30L) * (g_k3_offset_10cm - 20L)) / 70L);
      }

    /* K3 Cascade Tracking */
    g_motor_target_steps = K3Cascade_Process20ms(&g_k3_cascade, raw_pos,
                                                   g_ball_target_mm);
    g_motor_target_steps += (g_pa8_ff_val + g_pa11_ff_val);
    g_motor_target_steps += k3_mech_offset;
    return;
  }

  if ((BallTask_IsSequenceState() == 0U) &&
      (abs_error <= BALL_HOLD_ZONE_MM) && (abs_velocity <= BALL_HOLD_SPEED_MM))
  {
    g_ball_pid_integral = 0L;
    g_ball_pid_last_error = error;
    g_ball_task_pid_integral = 0L;
    g_ball_task_pid_last_error = error;
    g_motor_target_steps = App_AddAccelFeedforward(
      BallBalance_StaticSteps(g_ball_target_mm, (int32_t)MOTOR_STEPS_PER_REV));
    return;
  }

  g_ball_pid_integral += error;
  if (g_ball_pid_integral > BALL_PID_INTEGRAL_LIMIT)
  {
    g_ball_pid_integral = BALL_PID_INTEGRAL_LIMIT;
  }
  if (g_ball_pid_integral < -BALL_PID_INTEGRAL_LIMIT)
  {
    g_ball_pid_integral = -BALL_PID_INTEGRAL_LIMIT;
  }

  derivative = (int32_t)error - (int32_t)g_ball_pid_last_error;
  g_ball_pid_last_error = error;
  drive_sign = (BALL_CONTROL_SIGN * (int32_t)error >= 0L) ? 1L : -1L;

  control = BALL_CONTROL_SIGN *
            (((BALL_PID_KP_NUM * (int32_t)error) +
              (BALL_PID_KI_NUM * g_ball_pid_integral) +
              (BALL_PID_KD_NUM * derivative)) / BALL_PID_DEN);
  if (control == 0L)
  {
    control = BALL_CONTROL_SIGN * (int32_t)error;
  }
  abs_control = (control >= 0L) ? control : -control;

  if (abs_error <= BALL_SLOW_ZONE_MM)
  {
    if (abs_control < BALL_NEAR_MIN_STEPS)
    {
      abs_control = BALL_NEAR_MIN_STEPS;
    }
    if (abs_control > BALL_NEAR_MAX_STEPS)
    {
      abs_control = BALL_NEAR_MAX_STEPS;
    }
  }
  else if (abs_control < BALL_KICK_MIN_STEPS)
  {
    abs_control = BALL_KICK_MIN_STEPS;
  }
  if ((abs_error > BALL_SLOW_ZONE_MM) && (abs_velocity <= BALL_HOLD_SPEED_MM) &&
      (abs_control < BALL_STATIC_KICK_STEPS))
  {
    abs_control = BALL_STATIC_KICK_STEPS;
  }
  if (abs_control > BALL_KICK_MAX_STEPS)
  {
    abs_control = BALL_KICK_MAX_STEPS;
  }
  if ((abs_error <= BALL_MID_ZONE_MM) && (abs_control > BALL_MID_MAX_STEPS))
  {
    abs_control = BALL_MID_MAX_STEPS;
  }
  if ((abs_error <= BALL_SLOW_ZONE_MM) && (abs_velocity > BALL_HOLD_SPEED_MM) &&
      (abs_control > BALL_SLOW_MAX_STEPS))
  {
    abs_control = BALL_SLOW_MAX_STEPS;
  }

  /* 滚球控制：20mm内连续PID微调；20mm外短推、回平、观察，避免中远距离一直压角。 */
  start_pulse = 0U;
  pulse_ticks = BALL_KICK_PULSE_TICKS;
  coast_ticks = BALL_KICK_COAST_TICKS;

  if (abs_error <= BALL_PID_ZONE_MM)
  {
    Ball_ResetPulseControl();
    if (moving_toward_target != 0U)
    {
      if ((abs_error > dynamic_brake_distance) && (predicted_cross_target == 0U))
      {
        control = 0L;
      }
      else
      {
        brake_steps = abs_velocity * BALL_PID_KD_NUM / BALL_PID_DEN;
        brake_distance_error = dynamic_brake_distance - abs_error;
        if (brake_distance_error > 0L)
        {
          brake_steps += brake_distance_error * BALL_BRAKE_DIST_GAIN;
        }
        if ((abs_error <= BALL_SLOW_ZONE_MM) && (brake_steps < BALL_NEAR_BRAKE_MIN_STEPS))
        {
          brake_steps = BALL_NEAR_BRAKE_MIN_STEPS;
        }
        else if (brake_steps < BALL_BRAKE_MIN_STEPS)
        {
          brake_steps = BALL_BRAKE_MIN_STEPS;
        }
        if ((abs_error <= BALL_SLOW_ZONE_MM) && (brake_steps > BALL_NEAR_BRAKE_MAX_STEPS))
        {
          brake_steps = BALL_NEAR_BRAKE_MAX_STEPS;
        }
        if (brake_steps > BALL_BRAKE_MAX_STEPS)
        {
          brake_steps = BALL_BRAKE_MAX_STEPS;
        }
        control = -drive_sign * brake_steps;
      }
    }
    else
    {
      control = drive_sign * abs_control;
    }
  }
  else
  {
    if (moving_toward_target != 0U)
    {
      Ball_ResetPulseControl();
      if ((abs_error <= outer_brake_distance) || (predicted_cross_target != 0U))
      {
        brake_steps = abs_velocity * BALL_PID_KD_NUM / BALL_PID_DEN;
        brake_distance_error = outer_brake_distance - abs_error;
        if (brake_distance_error > 0L)
        {
          brake_steps += brake_distance_error * BALL_BRAKE_DIST_GAIN;
        }
        if (brake_steps < BALL_BRAKE_MIN_STEPS)
        {
          brake_steps = BALL_BRAKE_MIN_STEPS;
        }
        if (brake_steps > BALL_BRAKE_MAX_STEPS)
        {
          brake_steps = BALL_BRAKE_MAX_STEPS;
        }
        control = -drive_sign * brake_steps;
      }
      else
      {
        control = 0L;
      }
    }
    else
    {
      if ((g_ball_pulse_ticks == 0U) && (g_ball_pulse_coast_ticks == 0U))
      {
        start_pulse = 1U;
      }

      if (start_pulse != 0U)
      {
        g_ball_pulse_control_steps = drive_sign * abs_control;
        g_ball_pulse_ticks = pulse_ticks;
        g_ball_pulse_coast_reload = coast_ticks;
      }

      if (g_ball_pulse_ticks > 0U)
      {
        control = g_ball_pulse_control_steps;
        g_ball_pulse_ticks--;
        if (g_ball_pulse_ticks == 0U)
        {
          g_ball_pulse_coast_ticks = g_ball_pulse_coast_reload;
        }
      }
      else
      {
        control = 0L;
        if (g_ball_pulse_coast_ticks > 0U)
        {
          g_ball_pulse_coast_ticks--;
        }
      }
    }
  }

  control += BallBalance_StaticSteps(g_ball_target_mm, (int32_t)MOTOR_STEPS_PER_REV);
  control = App_AddAccelFeedforward(control);

  if (control > MOTOR_HARD_LIMIT_STEPS)
  {
    control = MOTOR_HARD_LIMIT_STEPS;
  }
  else if (control < -MOTOR_HARD_LIMIT_STEPS)
  {
    control = -MOTOR_HARD_LIMIT_STEPS;
  }

  delta_steps = control - g_motor_target_steps;
  slew_steps = (((control > 0L) && (g_motor_target_steps < 0L)) ||
                ((control < 0L) && (g_motor_target_steps > 0L))) ?
               BALL_BRAKE_SLEW_STEPS : BALL_TARGET_SLEW_STEPS;
  if (delta_steps > slew_steps)
  {
    control = g_motor_target_steps + slew_steps;
  }
  else if (delta_steps < -slew_steps)
  {
    control = g_motor_target_steps - slew_steps;
  }

  g_motor_target_steps = control;
}

static HAL_StatusTypeDef Mpu6050_Init(void)
{
  uint8_t value;

  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 2U, 20U) != HAL_OK)
  {
    g_mpu6050.ok = 0U;
    return HAL_ERROR;
  }

  value = 0x00U;
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U, 20U) != HAL_OK)
  {
    g_mpu6050.ok = 0U;
    return HAL_ERROR;
  }

  g_mpu6050.ok = 1U;
  return HAL_OK;
}

static void Mpu6050_Read(void)
{
  uint8_t raw[14];

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                       I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 20U) == HAL_OK)
  {
    g_mpu6050.ax = App_ReadInt16BE(&raw[0]);
    g_mpu6050.ay = App_ReadInt16BE(&raw[2]);
    g_mpu6050.az = App_ReadInt16BE(&raw[4]);
    g_mpu6050.temp = App_ReadInt16BE(&raw[6]);
    g_mpu6050.gx = App_ReadInt16BE(&raw[8]);
    g_mpu6050.gy = App_ReadInt16BE(&raw[10]);
    g_mpu6050.gz = App_ReadInt16BE(&raw[12]);
    g_mpu6050.ok = 1U;
    g_mpu6050.frames++;
  }
  else
  {
    g_mpu6050.ok = 0U;
  }
}

static int16_t App_IcmRawToMg(int16_t raw)
{
  int32_t mg = ((int32_t)raw * ICM42688_ACCEL_FS_MG) / 32768L;

  if (mg > 9999L)
  {
    mg = 9999L;
  }
  else if (mg < -9999L)
  {
    mg = -9999L;
  }

  return (int16_t)mg;
}

static int16_t App_IcmRawToDps(int16_t raw)
{
  int32_t dps = ((int32_t)raw * ICM42688_GYRO_FS_DPS) / 32768L;

  if (dps > 9999L)
  {
    dps = 9999L;
  }
  else if (dps < -9999L)
  {
    dps = -9999L;
  }

  return (int16_t)dps;
}

static int32_t App_GetMs901mLevelAxisCd(void)
{
  IMU901_Data_t imu;
  float angle;

  IMU901_GetData(&imu);
#if (MS901M_LEVEL_AXIS == MS901M_AXIS_ROLL)
  angle = imu.roll;
#else
  angle = imu.pitch;
#endif
  if (angle >= 0.0f)
  {
    return (int32_t)(angle * 100.0f + 0.5f);
  }
  return (int32_t)(angle * 100.0f - 0.5f);
}

static int16_t App_GetMs901mAccelAxisMg(const IMU901_Data_t *imu)
{
#if (MS901M_ACCEL_FF_AXIS == MS901M_ACCEL_AXIS_X)
  return imu->acc_x_mg;
#elif (MS901M_ACCEL_FF_AXIS == MS901M_ACCEL_AXIS_Y)
  return imu->acc_y_mg;
#else
  return imu->acc_z_mg;
#endif
}

static void App_UpdateMs901mAccelSnapshot(void)
{
  IMU901_Data_t imu;
  int32_t acc_mg;
  int32_t acc_mag_sq;
  int32_t max_mag_sq;
  int32_t jump_mg;

  IMU901_GetData(&imu);
  if (IMU901_HasFreshAccel(MS901M_Y_TIMEOUT_MS) == 0U)
  {
    g_ms901m_acc_ok = 0U;
    g_ms901m_acc_jump_count = 0U;
    return;
  }

  acc_mag_sq = ((int32_t)imu.acc_x_mg * (int32_t)imu.acc_x_mg) +
               ((int32_t)imu.acc_y_mg * (int32_t)imu.acc_y_mg) +
               ((int32_t)imu.acc_z_mg * (int32_t)imu.acc_z_mg);
  max_mag_sq = ACC_FF_MAX_VECTOR_MG * ACC_FF_MAX_VECTOR_MG;
  if (acc_mag_sq > max_mag_sq)
  {
    g_ms901m_acc_ok = 0U;
    g_ms901m_acc_jump_count = 0U;
    return;
  }

  acc_mg = (int32_t)App_GetMs901mAccelAxisMg(&imu);
  jump_mg = acc_mg - (int32_t)g_ms901m_axis_acc_filtered_mg;
  if (jump_mg < 0L)
  {
    jump_mg = -jump_mg;
  }

  if ((g_ms901m_acc_filter_valid != 0U) && (jump_mg > ACC_FF_MAX_JUMP_MG))
  {
    if (g_ms901m_acc_jump_count < ACC_FF_JUMP_ACCEPT_COUNT)
    {
      g_ms901m_acc_jump_count++;
      return;
    }
  }
  else
  {
    g_ms901m_acc_jump_count = 0U;
  }

  g_ms901m_axis_acc_mg = (int16_t)acc_mg;
  if ((g_ms901m_acc_filter_valid == 0U) ||
      (g_ms901m_acc_jump_count >= ACC_FF_JUMP_ACCEPT_COUNT))
  {
    /* 连续大变化视为真实平移加速度/姿态变化，避免滤波器卡在旧值。 */
    g_ms901m_axis_acc_filtered_mg = (int16_t)acc_mg;
    g_ms901m_acc_filter_valid = 1U;
    g_ms901m_acc_jump_count = 0U;
  }
  else
  {
    g_ms901m_axis_acc_filtered_mg =
      (int16_t)(((int32_t)g_ms901m_axis_acc_filtered_mg * ACC_FF_FILTER_NUM + acc_mg) /
                ACC_FF_FILTER_DEN);
  }
  if (g_accel_ff_zero_valid != 0U)
  {
    g_ms901m_acc_error_mg =
      (int16_t)((int32_t)g_ms901m_axis_acc_filtered_mg - (int32_t)g_accel_ff_zero_mg);
  }
  else
  {
    g_ms901m_acc_error_mg = g_ms901m_axis_acc_filtered_mg;
  }
  g_ms901m_acc_ok = 1U;
}

static int32_t App_Ms901mAngleDiffCd(int32_t current_cd, int32_t zero_cd)
{
  int32_t diff = current_cd - zero_cd;

  /* 欧拉角是环形量，跨过±180/360度边界时必须取最短角度差。 */
  if (diff > (MS901M_CDEG_PER_REV / 2L))
  {
    diff -= MS901M_CDEG_PER_REV;
  }
  else if (diff < -(MS901M_CDEG_PER_REV / 2L))
  {
    diff += MS901M_CDEG_PER_REV;
  }
  return diff;
}

static int32_t ImuLevel_ApplyMs901mAngleFeedback(int32_t outer_target_steps)
{
  int32_t target_offset_cd;
  int32_t target_angle_cd;
  int32_t current_angle_cd;
  int32_t angle_error_cd;
  int32_t abs_angle_error_cd;
  int32_t correction_steps;

  if (g_ms901m_level_zero_valid == 0U)
  {
    return outer_target_steps;
  }

  /* 外环步数先按机构1:6换成期望水管Roll；反馈项只补真实角与目标角的偏差。 */
  target_offset_cd =
    (MS901M_LEVEL_SIGN * outer_target_steps * MS901M_CDEG_PER_REV) /
    ((int32_t)MOTOR_STEPS_PER_REV * MS901M_LEVEL_PIPE_RATIO_DEN);
  target_angle_cd = g_ms901m_level_zero_cd + target_offset_cd;
  current_angle_cd = App_GetMs901mLevelAxisCd();
  angle_error_cd = App_Ms901mAngleDiffCd(target_angle_cd, current_angle_cd);
  g_ms901m_level_error_cd = angle_error_cd;
  abs_angle_error_cd = (angle_error_cd >= 0L) ? angle_error_cd : -angle_error_cd;

  if (abs_angle_error_cd <= MS901M_LEVEL_DEADBAND_CD)
  {
    return outer_target_steps;
  }

  correction_steps =
    (MS901M_LEVEL_SIGN * angle_error_cd * (int32_t)MOTOR_STEPS_PER_REV *
     MS901M_LEVEL_GAIN_NUM) /
    (MS901M_CDEG_PER_REV * MS901M_LEVEL_GAIN_DEN);
  if (correction_steps > MS901M_LEVEL_MAX_CORRECT_STEPS)
  {
    correction_steps = MS901M_LEVEL_MAX_CORRECT_STEPS;
  }
  else if (correction_steps < -MS901M_LEVEL_MAX_CORRECT_STEPS)
  {
    correction_steps = -MS901M_LEVEL_MAX_CORRECT_STEPS;
  }

  return outer_target_steps + correction_steps;
}

static void App_UpdateIcmSnapshot(void)
{
  ICM42688_Data_t icm;
  int32_t acc_mg;
  int32_t ay_mg;
  int32_t az_mg;
  int32_t acc_mag_sq;
  int32_t max_mag_sq;
  int32_t jump_mg;

  ICM42688_GetData(&icm);
  g_icm_ok = ((icm.ok != 0U) && (icm.who_am_i == 0x47U)) ? 1U : 0U;
  if (g_icm_ok != 0U)
  {
    acc_mg = App_IcmRawToMg(icm.ax);
    ay_mg = App_IcmRawToMg(icm.ay);
    az_mg = App_IcmRawToMg(icm.az);
    acc_mag_sq = (acc_mg * acc_mg) + (ay_mg * ay_mg) + (az_mg * az_mg);
    max_mag_sq = ACC_FF_MAX_VECTOR_MG * ACC_FF_MAX_VECTOR_MG;
    jump_mg = acc_mg - (int32_t)g_icm_axis_acc_filtered_mg;
    if (jump_mg < 0L)
    {
      jump_mg = -jump_mg;
    }

    if ((acc_mag_sq <= max_mag_sq) &&
        ((g_icm_acc_filter_valid == 0U) || (jump_mg <= ACC_FF_MAX_JUMP_MG)))
    {
      g_icm_axis_acc_mg = (int16_t)acc_mg;
      if (g_icm_acc_filter_valid == 0U)
      {
        g_icm_axis_acc_filtered_mg = (int16_t)acc_mg;
        g_icm_acc_filter_valid = 1U;
      }
      else
      {
        g_icm_axis_acc_filtered_mg = (int16_t)(((int32_t)g_icm_axis_acc_filtered_mg * ACC_FF_FILTER_NUM + acc_mg) /
                                               ACC_FF_FILTER_DEN);
      }
    }
  }
  else
  {
    g_icm_acc_filter_valid = 0U;
    g_icm_acc_jump_count = 0U;
  }
}

static int32_t App_AddAccelFeedforward(int32_t control_steps)
{
#if (APP_ACCEL_FEEDFORWARD_ENABLE == 0U)
  g_accel_ff_steps = 0L;
  g_ms901m_acc_error_mg = 0;
  return control_steps;
#else
  int32_t acc_mg;
  int32_t ff_steps;
  int32_t ff_limit_steps;

  App_UpdateMs901mAccelSnapshot();
  if ((g_ms901m_acc_ok == 0U) || (g_ms901m_acc_filter_valid == 0U) || (g_accel_ff_zero_valid == 0U))
  {
    g_accel_ff_steps = 0L;
    g_ms901m_acc_error_mg = 0;
    return control_steps;
  }

  acc_mg = (int32_t)g_ms901m_axis_acc_filtered_mg - (int32_t)g_accel_ff_zero_mg;
  if ((acc_mg > -ACC_FF_DEADBAND_MG) && (acc_mg < ACC_FF_DEADBAND_MG))
  {
    acc_mg = 0L;
  }

  /* theta≈a/g先得到水管目标角；机构1:6，电机目标角必须乘6再换步数。 */
  ff_steps = (ACC_FF_SIGN * acc_mg * (int32_t)MOTOR_STEPS_PER_REV *
              MS901M_LEVEL_PIPE_RATIO_DEN) /
             ACC_FF_SMALL_ANGLE_DEN;
  ff_steps = (ff_steps * ACC_FF_GAIN_NUM) / ACC_FF_GAIN_DEN;
  ff_limit_steps = (int32_t)((MOTOR_STEPS_PER_REV * ACC_FF_MAX_DEG + 180L) / 360L);
  if (ff_steps > ff_limit_steps)
  {
    ff_steps = ff_limit_steps;
  }
  else if (ff_steps < -ff_limit_steps)
  {
    ff_steps = -ff_limit_steps;
  }

  ff_steps = ((g_accel_ff_steps * ACC_FF_OUTPUT_FILTER_NUM) + ff_steps) /
             ACC_FF_OUTPUT_FILTER_DEN;
  if ((ff_steps > -ACC_FF_OUTPUT_DEADBAND_STEPS) &&
      (ff_steps < ACC_FF_OUTPUT_DEADBAND_STEPS))
  {
    ff_steps = 0L;
  }

  g_accel_ff_steps = ff_steps;
  g_ms901m_acc_error_mg = (int16_t)acc_mg;
  return control_steps + ff_steps;
#endif
}

static void Oled_ShowLine(uint8_t row, const char *text)
{
  char padded[17];
  uint8_t i;

  memset(padded, ' ', 16U);
  padded[16] = '\0';
  if (text != NULL)
  {
    for (i = 0U; (i < 16U) && (text[i] != '\0'); i++)
    {
      padded[i] = text[i];
    }
  }
  OLED_ShowString(row, 1U, padded);
}

static void Oled_DrawStatus(void)
{
  char line[20];

  if (g_ball_task_state == BALL_TASK_DEBUG_FF)
  {
    if (g_debug_page == 0U)
    {
      (void)snprintf(line, sizeof(line), "PA8 S:%+04ld %c", (long)g_pa8_ff_start_steps, g_debug_cursor == 0 ? '<' : ' ');
      Oled_ShowLine(1U, line);
      (void)snprintf(line, sizeof(line), "PA8 T:%04ums %c", (unsigned int)g_pa8_ff_start_duration_ms, g_debug_cursor == 1 ? '<' : ' ');
      Oled_ShowLine(2U, line);
      (void)snprintf(line, sizeof(line), "PA11S:%+04ld %c", (long)g_pa11_ff_start_steps, g_debug_cursor == 2 ? '<' : ' ');
      Oled_ShowLine(3U, line);
      (void)snprintf(line, sizeof(line), "PA11T:%04ums %c", (unsigned int)g_pa11_ff_start_duration_ms, g_debug_cursor == 3 ? '<' : ' ');
      Oled_ShowLine(4U, line);
    }
    else if (g_debug_page == 1U)
    {
      (void)snprintf(line, sizeof(line), "PA8ES:%+04ld %c", (long)g_pa8_ff_stop_steps, g_debug_cursor == 0 ? '<' : ' ');
      Oled_ShowLine(1U, line);
      (void)snprintf(line, sizeof(line), "PA8ET:%04ums %c", (unsigned int)g_pa8_ff_stop_duration_ms, g_debug_cursor == 1 ? '<' : ' ');
      Oled_ShowLine(2U, line);
      (void)snprintf(line, sizeof(line), "P11ES:%+04ld %c", (long)g_pa11_ff_stop_steps, g_debug_cursor == 2 ? '<' : ' ');
      Oled_ShowLine(3U, line);
      (void)snprintf(line, sizeof(line), "P11ET:%04ums %c", (unsigned int)g_pa11_ff_stop_duration_ms, g_debug_cursor == 3 ? '<' : ' ');
      Oled_ShowLine(4U, line);
    }
    else
    {
      (void)snprintf(line, sizeof(line), "CENTER:%+03ldcm %c", (long)(g_ball_target_mm / 10L), g_debug_cursor == 0 ? '<' : ' ');
      Oled_ShowLine(1U, line);
      (void)snprintf(line, sizeof(line), "K3 OFF:%+04ld %c", (long)g_k3_offset_10cm, g_debug_cursor == 1 ? '<' : ' ');
      Oled_ShowLine(2U, line);
      Oled_ShowLine(3U, " ");
      Oled_ShowLine(4U, " ");
    }
    return;
  }
#if (APP_MS901M_ENABLE != 0U)
  IMU901_Data_t imu;
#else
  ICM42688_Data_t icm;
#endif
  uint32_t k230_age;
  uint32_t task_elapsed;
  int32_t imu_error;

#if (APP_MS901M_ENABLE != 0U)
  IMU901_GetData(&imu);
  App_UpdateMs901mAccelSnapshot();
  {
    int32_t acc_show_mg = (g_accel_ff_zero_valid != 0U) ?
                          (int32_t)g_ms901m_acc_error_mg :
                          (int32_t)g_ms901m_axis_acc_filtered_mg;
    (void)snprintf(line, sizeof(line), "M%+05ld F%04lu",
                     (long)acc_show_mg,
                     (unsigned long)(imu.accel_frames % 10000UL));
    }
    if ((g_ball_task_state != BALL_TASK_HOLD_CENTER) &&
        (g_ball_task_state != BALL_TASK_GO_POS) &&
        (g_ball_task_state != BALL_TASK_GO_NEG) &&
        (g_ball_task_state != BALL_TASK_HOLD_NEG))
    {
      Oled_ShowLine(1U, line);
    }

  if ((g_after_home_mode == HOME_AFTER_IMU_LEVEL) ||
      (g_imu_zero_pending != 0U) || (g_imu_level_active != 0U))
  {
    int32_t acc_show_mg = (g_accel_ff_zero_valid != 0U) ?
                          (int32_t)g_ms901m_acc_error_mg :
                          (int32_t)g_ms901m_axis_acc_filtered_mg;
    if (IMU901_HasFreshAccel(MS901M_Y_TIMEOUT_MS) == 0U)
    {
      (void)snprintf(line, sizeof(line), "MS901 WAIT ACC");
    }
    else if (g_imu_zero_pending != 0U)
    {
      (void)snprintf(line, sizeof(line), "ZERO %02u/%02u",
                     (unsigned int)g_imu_zero_count,
                     (unsigned int)IMU_LEVEL_ZERO_SAMPLES);
    }
    else if (g_imu_level_active != 0U)
    {
      (void)snprintf(line, sizeof(line), "RUN P%+05ld",
                     (long)g_motor_pos_steps);
    }
    else
    {
      (void)snprintf(line, sizeof(line), "WAIT HOME");
    }
    Oled_ShowLine(2U, line);
    (void)snprintf(line, sizeof(line), "Z%+05d M%+05ld",
                   (int)g_accel_ff_zero_mg, (long)acc_show_mg);
    Oled_ShowLine(3U, line);
    (void)snprintf(line, sizeof(line), "T:%+04ldmm G:%+04ld",
                   (long)g_ball_target_mm, (long)g_motor_target_steps);
    Oled_ShowLine(4U, line);
    return;
  }

  if ((g_manual_target_adjust_enabled != 0U) &&
        ((g_after_home_mode == HOME_AFTER_BALL_CENTER) ||
         (g_ball_task_state == BALL_TASK_WAIT_HOME) ||
         (g_ball_task_state == BALL_TASK_HOLD_CENTER)))
    {
      
#if (APP_K230_MONITOR_ONLY != 0U)
    if (AppK230_IsTargetValid() != 0U)
    {
      (void)snprintf(line, sizeof(line), "B:%+04d T:%+04ld",
                     (int)AppK230_GetPosMm(), (long)g_ball_target_mm);
    }
    else
    {
      (void)snprintf(line, sizeof(line), "NO BALL T:%+04ld",
                     (long)g_ball_target_mm);
    }
#else
    (void)snprintf(line, sizeof(line), "VIS T:%+04ld", (long)g_ball_target_mm);
#endif
    Oled_ShowLine(2U, line);
    (void)snprintf(line, sizeof(line), "G:%+04ld P:%+04ld",
                   (long)g_motor_target_steps, (long)g_motor_pos_steps);
    Oled_ShowLine(3U, line);
    (void)snprintf(line, sizeof(line), "PA8: %c  PA11: %c",
                   g_pa8_edge, g_pa11_edge);
    Oled_ShowLine(4U, line);
    return;
  }

  if (g_ball_task_state == BALL_TASK_IDLE)
  {
    if (IMU901_HasFreshAccel(MS901M_Y_TIMEOUT_MS) == 0U)
    {
      Oled_ShowLine(2U, "MS901 ACC WAIT");
    }
    else
    {
      Oled_ShowLine(2U, "MS901 RX OK");
    }
    Oled_ShowLine(3U, "USART1 115200");
    Oled_ShowLine(4U, "K1 HOME+LEVEL");
    return;
  }
#else
  ICM42688_GetData(&icm);

  if (g_icm_ok == 0U)
  {
    (void)snprintf(line, sizeof(line), "ID:%02X ICM ERR  ",
                   (unsigned int)icm.who_am_i);
    Oled_ShowLine(1U, line);
    (void)snprintf(line, sizeof(line), "M0:%02X M1:%02X   ",
                   (unsigned int)icm.probe_id[0], (unsigned int)icm.probe_id[1]);
    Oled_ShowLine(2U, line);
    (void)snprintf(line, sizeof(line), "M2:%02X M3:%02X   ",
                   (unsigned int)icm.probe_id[2], (unsigned int)icm.probe_id[3]);
    Oled_ShowLine(3U, line);
    (void)snprintf(line, sizeof(line), "E%08lu S%02X  ",
                   (unsigned long)icm.errors, (unsigned int)icm.spi_status);
    Oled_ShowLine(4U, line);
    return;
  }

  (void)snprintf(line, sizeof(line), "AX%+5d R%+6d",
                   (int)App_IcmRawToMg(icm.ax),
                   (int)icm.ax);
    if ((g_ball_task_state != BALL_TASK_HOLD_CENTER) &&
        (g_ball_task_state != BALL_TASK_GO_POS) &&
        (g_ball_task_state != BALL_TASK_GO_NEG) &&
        (g_ball_task_state != BALL_TASK_HOLD_NEG))
    {
      Oled_ShowLine(1U, line);
    }
#endif

#if (APP_K230_MONITOR_ONLY != 0U)
  k230_age = AppK230_GetAgeMs();
  if (AppK230_IsTargetValid() == 0U)
  {
    (void)snprintf(line, sizeof(line), "N:%04lu P:%+04ld",
                   (unsigned long)(k230_age > 9999UL ? 9999UL : k230_age),
                   (long)g_motor_pos_steps);
    Oled_ShowLine(2U, line);
    (void)snprintf(line, sizeof(line), "T:%010lu ",
                     (unsigned long)AppK230_GetFrames());
      Oled_ShowLine(3U, line);
      (void)snprintf(line, sizeof(line), "T:%+04ldmm NO BALL",
                     (long)g_ball_target_mm);
      Oled_ShowLine(4U, line);
      return;
    }

    if ((g_ball_task_state == BALL_TASK_GO_POS) ||
        (g_ball_task_state == BALL_TASK_GO_NEG) ||
        (g_ball_task_state == BALL_TASK_HOLD_NEG))
    {
      (void)snprintf(line, sizeof(line), "K2:%+03ld/%+03ld", (long)g_k2_offset_go_pos, (long)g_k2_offset_go_neg);
      Oled_ShowLine(1U, line);
    }

    (void)snprintf(line, sizeof(line), "M:%+04d P:%+04ld",
                   (int)AppK230_GetPosMm(), (long)g_motor_pos_steps);
    Oled_ShowLine(2U, line);

  static uint32_t k2_end_tick = 0;
  if ((g_ball_task_state == BALL_TASK_GO_POS) ||
      (g_ball_task_state == BALL_TASK_GO_NEG))
  {
    k2_end_tick = HAL_GetTick();
  }

  if ((g_ball_task_state == BALL_TASK_GO_POS) ||
      (g_ball_task_state == BALL_TASK_GO_NEG) ||
      (g_ball_task_state == BALL_TASK_HOLD_NEG) ||
      (g_ball_task_state == BALL_TASK_FINISHED))
  {
    task_elapsed = k2_end_tick - g_ball_task_start_tick;
  }
  else
  {
    task_elapsed = 0UL;
  }
  if (task_elapsed > BALL_TASK_TOTAL_LIMIT_MS)
  {
    task_elapsed = BALL_TASK_TOTAL_LIMIT_MS;
  }
  (void)snprintf(line, sizeof(line), "Time: %04lu ms  ",
                 (unsigned long)task_elapsed);
  Oled_ShowLine(3U, line);

  (void)snprintf(line, sizeof(line), "T:%+04ldmm E:%+04ld",
                 (long)g_ball_target_mm,
                 (long)(g_ball_target_mm - (int32_t)AppK230_GetPosMm()));
  Oled_ShowLine(4U, line);
  return;
#else
  (void)snprintf(line, sizeof(line), "AX:%+05dmg     ",
                 (int)App_IcmRawToMg(icm.ax));
  Oled_ShowLine(2U, line);

  (void)snprintf(line, sizeof(line), "AY:%+05dmg     ",
                 (int)App_IcmRawToMg(icm.ay));
  Oled_ShowLine(3U, line);

  (void)snprintf(line, sizeof(line), "AZ:%+05dmg     ",
                 (int)App_IcmRawToMg(icm.az));
  Oled_ShowLine(4U, line);
#endif
}

static void App_Init(void)
{
  App_InitRuntimeKeys();
  Motor_Enable(0U);
  Motor_Stop();
  g_motor_disable_request = 1U;

#if (APP_ICM42688_ENABLE != 0U)
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 8U, 0U);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  (void)ICM42688_Init();
  App_UpdateIcmSnapshot();
#endif
#if (APP_MPU6050_ENABLE != 0U)
  (void)Mpu6050_Init();
#endif
  if (OLED_Init() == HAL_OK)
  {
    Oled_DrawStatus();
  }

#if (APP_MS901M_ENABLE != 0U)
  {
    (void)IMU901_Init(&huart1);
    (void)IMU901_EnsureAccelRange2G();
    (void)IMU901_SetAngleOutput100Hz();
  }
#endif

#if (APP_K230_MONITOR_ONLY != 0U)
  {
    AppK230_Init(&huart2);
    AppK230_StartRx();
  }
#endif
}

static void App_Process1ms(void)
{
  static uint16_t led_tick;
  static uint16_t control_tick;
  static uint16_t mpu_tick;
  static uint16_t oled_tick;
  static uint8_t k1_stable = GPIO_PIN_SET;
  static uint8_t k1_last = GPIO_PIN_SET;
  static uint16_t k1_debounce;
  static uint8_t k2_stable = GPIO_PIN_SET;
  static uint8_t k2_last = GPIO_PIN_SET;
  static uint16_t k2_debounce;
  static uint8_t k3_stable = GPIO_PIN_SET;
  static uint8_t k3_last = GPIO_PIN_SET;
  static uint16_t k3_debounce;
  static uint8_t k4_stable = GPIO_PIN_SET;
  static uint8_t k4_last = GPIO_PIN_SET;
  static uint16_t k4_debounce;
  static uint8_t k5_stable = GPIO_PIN_SET;
  static uint8_t k5_last = GPIO_PIN_SET;
  static uint16_t k5_debounce;
  static uint8_t k6_stable = GPIO_PIN_SET;
  static uint8_t k6_last = GPIO_PIN_SET;
  static uint16_t k6_debounce;
  uint16_t led_period;

#if (APP_MS901M_ENABLE != 0U)
  {
    IMU901_Process();
  }
#endif

#if (APP_K230_MONITOR_ONLY != 0U)
  {
    /* K230无目标只更新视觉状态；不能在公共1ms路径抢占K1的MS901M前馈。 */
    (void)AppK230_ProcessLine();
  }
#endif

  if (App_ReadKeyPress(RUNTIME_KEY_K1_GPIO_Port, RUNTIME_KEY_K1_Pin, &k1_stable, &k1_last, &k1_debounce) != 0U)
    {
      if (g_ball_task_state == BALL_TASK_DEBUG_FF)
      {
        g_debug_page = (g_debug_page + 1U) % 3U;
      }
      else if ((g_ball_task_state == BALL_TASK_GO_POS) || (g_ball_task_state == BALL_TASK_GO_NEG) || (g_ball_task_state == BALL_TASK_HOLD_NEG))
      {
        g_k2_offset_go_pos += 1L;
        g_k2_offset_go_neg += 1L;
      }
      else
      {
        g_k4_next_disable = 0U;
        ImuLevel_RequestHome();
      }
    }

  if (App_ReadKeyPress(RUNTIME_KEY_K2_GPIO_Port, RUNTIME_KEY_K2_Pin, &k2_stable, &k2_last, &k2_debounce) != 0U)
  {
    /* K2：中心稳定后开始赛题3，+5cm到位后折返到-5cm并保持。 */
    g_k4_next_disable = 0U;
    BallTask_StartSequence();
  }

  (void)App_ReadKeyPress(RUNTIME_KEY_K3_GPIO_Port, RUNTIME_KEY_K3_Pin, &k3_stable, &k3_last, &k3_debounce);
    static uint32_t k3_press_tick = 0;
    static uint8_t k3_long_pressed = 0;
    
    if (k3_stable == 0U)
    {
      if (k3_press_tick == 0) k3_press_tick = HAL_GetTick();
      if ((k3_long_pressed == 0) && (HAL_GetTick() - k3_press_tick > 1000))
      {
        k3_long_pressed = 1;
        if (g_ball_task_state == BALL_TASK_DEBUG_FF)
        {
          g_ball_task_state = BALL_TASK_WAIT_HOME;
          g_motor_home_request = 1U;
        }
        else
        {
          g_ball_task_state = BALL_TASK_DEBUG_FF;
        }
      }
    }
    else
    {
      if (k3_press_tick != 0)
      {
        if (k3_long_pressed == 0)
        {
          if (g_ball_task_state != BALL_TASK_DEBUG_FF)
          {
            g_k4_next_disable = 0U;
            BallTask_RequestCenterHome();
          }
        }
        k3_press_tick = 0;
        k3_long_pressed = 0;
      }
    }

  if (App_ReadKeyPress(RUNTIME_KEY_K4_GPIO_Port, RUNTIME_KEY_K4_Pin, &k4_stable, &k4_last, &k4_debounce) != 0U)
    {
      if (g_ball_task_state == BALL_TASK_DEBUG_FF)
      {
        if (g_debug_page == 2U) {
            g_debug_cursor = (g_debug_cursor + 1) % 2;
        } else {
            g_debug_cursor = (g_debug_cursor + 1) % 4;
        }
      }
      else if ((g_ball_task_state == BALL_TASK_GO_POS) || (g_ball_task_state == BALL_TASK_GO_NEG) || (g_ball_task_state == BALL_TASK_HOLD_NEG))
      {
        g_k2_offset_go_pos -= 1L;
        g_k2_offset_go_neg -= 1L;
      }
      else
      {
        if (g_k4_next_disable == 0U)
        {
          HomeOnly_Request();
        }
        else
        {
          g_k4_next_disable = 0U;
          g_motor_home_request = 0U;
          g_motor_home_pending = 0U;
          g_balance_start_after_home_pending = 0U;
          g_after_home_mode = HOME_AFTER_NONE;
          g_manual_target_adjust_enabled = 0U;
          g_motor_disable_request = 1U;
        }
      }
    }

  {
      static uint32_t k5_press_tick = 0U;
      static uint32_t k5_repeat_tick = 0U;
      uint8_t k5_trigger = 0U;

      if (App_ReadKeyPress(RUNTIME_KEY_K5_GPIO_Port, RUNTIME_KEY_K5_Pin, &k5_stable, &k5_last, &k5_debounce) != 0U)
      {
        k5_trigger = 1U;
        k5_press_tick = HAL_GetTick();
      }
      else if (k5_stable == GPIO_PIN_RESET)
      {
        if (HAL_GetTick() - k5_press_tick > 500U)
        {
          if (HAL_GetTick() - k5_repeat_tick > 100U)
          {
            k5_repeat_tick = HAL_GetTick();
            k5_trigger = 1U;
          }
        }
      }

      if (k5_trigger != 0U)
      {
        if (g_ball_task_state == BALL_TASK_DEBUG_FF)
        {
          if (g_debug_page == 0U)
          {
            if (g_debug_cursor == 0) g_pa8_ff_start_steps += 5L;
            else if (g_debug_cursor == 1) g_pa8_ff_start_duration_ms += 50U;
            else if (g_debug_cursor == 2) g_pa11_ff_start_steps += 5L;
            else if (g_debug_cursor == 3) g_pa11_ff_start_duration_ms += 50U;
          }
          else if (g_debug_page == 1U)
          {
            if (g_debug_cursor == 0) g_pa8_ff_stop_steps += 5L;
            else if (g_debug_cursor == 1) g_pa8_ff_stop_duration_ms += 50U;
            else if (g_debug_cursor == 2) g_pa11_ff_stop_steps += 5L;
            else if (g_debug_cursor == 3) g_pa11_ff_stop_duration_ms += 50U;
          }
          else
          {
            if (g_debug_cursor == 0) {
              g_ball_target_mm += 10L;
              if (g_ball_target_mm > 100L) g_ball_target_mm = 100L;
            } else if (g_debug_cursor == 1) {
              g_k3_offset_10cm += 1L;
            }
          }
        }
        else
        {
          BallTarget_AdjustByMm(BALL_TARGET_KEY_STEP_MM);
        }
      }
    }

  {
      static uint32_t k6_press_tick = 0U;
      static uint32_t k6_repeat_tick = 0U;
      uint8_t k6_trigger = 0U;

      if (App_ReadKeyPress(RUNTIME_KEY_K6_GPIO_Port, RUNTIME_KEY_K6_Pin, &k6_stable, &k6_last, &k6_debounce) != 0U)
      {
        k6_trigger = 1U;
        k6_press_tick = HAL_GetTick();
      }
      else if (k6_stable == GPIO_PIN_RESET)
      {
        if (HAL_GetTick() - k6_press_tick > 500U)
        {
          if (HAL_GetTick() - k6_repeat_tick > 100U)
          {
            k6_repeat_tick = HAL_GetTick();
            k6_trigger = 1U;
          }
        }
      }

      if (k6_trigger != 0U)
      {
        if (g_ball_task_state == BALL_TASK_DEBUG_FF)
        {
          if (g_debug_page == 0U)
          {
            if (g_debug_cursor == 0) g_pa8_ff_start_steps -= 5L;
            else if (g_debug_cursor == 1) { if(g_pa8_ff_start_duration_ms >= 50) g_pa8_ff_start_duration_ms -= 50U; }
            else if (g_debug_cursor == 2) g_pa11_ff_start_steps -= 5L;
            else if (g_debug_cursor == 3) { if(g_pa11_ff_start_duration_ms >= 50) g_pa11_ff_start_duration_ms -= 50U; }
          }
          else if (g_debug_page == 1U)
          {
            if (g_debug_cursor == 0) g_pa8_ff_stop_steps -= 5L;
            else if (g_debug_cursor == 1) { if(g_pa8_ff_stop_duration_ms >= 50) g_pa8_ff_stop_duration_ms -= 50U; }
            else if (g_debug_cursor == 2) g_pa11_ff_stop_steps -= 5L;
            else if (g_debug_cursor == 3) { if(g_pa11_ff_stop_duration_ms >= 50) g_pa11_ff_stop_duration_ms -= 50U; }
          }
          else
          {
            if (g_debug_cursor == 0) {
              g_ball_target_mm -= 10L;
              if (g_ball_target_mm < -100L) g_ball_target_mm = -100L;
            } else if (g_debug_cursor == 1) {
              g_k3_offset_10cm -= 1L;
            }
          }
        }
        else
        {
          BallTarget_AdjustByMm(-BALL_TARGET_KEY_STEP_MM);
        }
      }
    }

  {
    uint8_t pa8_curr = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
    if (pa8_curr != g_pa8_last_state)
    {
      g_pa8_last_state = pa8_curr;
      g_pa8_edge = (pa8_curr == GPIO_PIN_SET) ? '^' : 'v';
      g_pa8_ff_duration = (pa8_curr == GPIO_PIN_SET) ? g_pa8_ff_start_duration_ms : g_pa8_ff_stop_duration_ms;
        g_pa8_ff_timer = g_pa8_ff_duration;
        g_pa8_ff_target_val = (pa8_curr == GPIO_PIN_SET) ? g_pa8_ff_start_steps : g_pa8_ff_stop_steps;
    }
    if (g_pa8_ff_timer > 0U)
    {
      g_pa8_ff_timer--;
      if (g_pa8_ff_duration > 0U)
      {
        g_pa8_ff_val = g_pa8_ff_target_val * (int32_t)g_pa8_ff_timer / (int32_t)g_pa8_ff_duration;
      }
    }
    else
    {
      g_pa8_ff_val = 0L;
    }

    uint8_t pa11_curr = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
    if (pa11_curr != g_pa11_last_state)
    {
      g_pa11_last_state = pa11_curr;
      g_pa11_edge = (pa11_curr == GPIO_PIN_SET) ? '^' : 'v';
      g_pa11_ff_duration = (pa11_curr == GPIO_PIN_SET) ? g_pa11_ff_start_duration_ms : g_pa11_ff_stop_duration_ms;
        g_pa11_ff_timer = g_pa11_ff_duration;
        g_pa11_ff_target_val = (pa11_curr == GPIO_PIN_SET) ? g_pa11_ff_start_steps : g_pa11_ff_stop_steps;
    }
    if (g_pa11_ff_timer > 0U)
    {
      g_pa11_ff_timer--;
      if (g_pa11_ff_duration > 0U)
      {
        g_pa11_ff_val = g_pa11_ff_target_val * (int32_t)g_pa11_ff_timer / (int32_t)g_pa11_ff_duration;
      }
    }
    else
    {
      g_pa11_ff_val = 0L;
    }
  }

  SwingTest_Process1ms();
  BalanceMotor_Process1ms();

  control_tick++;
  if (control_tick >= 20U)
  {
    control_tick = 0U;
    Tracking_Process20ms();
  }

  mpu_tick++;
  if ((APP_MPU6050_ENABLE != 0U) && (mpu_tick >= 20U))
  {
    mpu_tick = 0U;
    g_mpu_read_request = 1U;
  }

  oled_tick++;
  if (oled_tick >= 200U)
  {
    oled_tick = 0U;
    g_oled_update_request = 1U;
  }

  led_period = ((g_tracking_enabled != 0U) && (AppK230_IsTargetValid() == 0U)) ?
               LED_TRACK_TOGGLE_TICKS : LED_TOGGLE_TICKS;
  led_tick++;
  if (led_tick >= led_period)
  {
    led_tick = 0U;
    HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);
  }
}

static void App_Background(void)
{
  if (g_motor_disable_request != 0U)
  {
    g_motor_disable_request = 0U;
    Motor_RequestSerialDisable();
  }

  if (g_motor_home_request != 0U)
  {
    g_motor_home_request = 0U;
    Motor_RequestHome();
  }

  if ((g_motor_disable_pending != 0U) &&
      ((g_motor_disable_last_tx_tick == 0U) ||
       ((HAL_GetTick() - g_motor_disable_last_tx_tick) >= EMM_DISABLE_REPEAT_MS)))
  {
    g_motor_disable_last_tx_tick = HAL_GetTick();
    if (Motor_SendSerialEnable(0U) == HAL_OK)
    {
      g_motor_disable_sent_count++;
    }
    g_motor_disable_pending--;
  }

  if ((g_motor_home_pending != 0U) &&
      ((HAL_GetTick() - g_motor_home_tick) >= EMM_HOME_DELAY_MS))
  {
    if (Motor_SendSerialHome() == HAL_OK)
    {
      g_motor_home_sent_count++;
    }
    g_balance_start_after_home_tick = HAL_GetTick();
    g_motor_home_pending = 0U;
  }

  if ((g_balance_start_after_home_pending != 0U) &&
      (g_motor_home_pending == 0U) &&
      ((HAL_GetTick() - g_balance_start_after_home_tick) >= BALANCE_START_AFTER_HOME_MS))
  {
    g_balance_start_after_home_pending = 0U;
    if (g_after_home_mode == HOME_AFTER_IMU_LEVEL)
    {
      ImuLevel_StartZeroAfterHome();
    }
    else if (g_after_home_mode == HOME_AFTER_BALL_CENTER)
    {
      BallTask_StartCenterAfterHome();
    }
    g_after_home_mode = HOME_AFTER_NONE;
  }

#if (APP_ICM42688_ENABLE != 0U)
  ICM42688_Process();
  App_UpdateIcmSnapshot();
#elif (APP_MS901M_ENABLE != 0U)
  App_UpdateMs901mAccelSnapshot();
#endif

#if (APP_MPU6050_ENABLE != 0U)
  if (g_mpu_read_request != 0U)
  {
    g_mpu_read_request = 0U;
    Mpu6050_Read();
  }
#endif

  if (g_oled_update_request != 0U)
  {
    g_oled_update_request = 0U;
    if (OLED_IsOk() != 0U)
    {
      Oled_DrawStatus();
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  App_Init();

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Background();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    App_Process1ms();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  IMU901_UART_RxCpltCallback(huart);
  AppK230_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  IMU901_UART_ErrorCallback(huart);
  AppK230_ErrorCallback(huart);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  ICM42688_EXTI_Callback(GPIO_Pin);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
