#include "LineTracking.h"
#include "Timer1.h"

/*
 * 传感器顺序：
 * 从车后向车前看，x1 在最左，x8 在最右。
 * 模块返回 0 表示检测到黑线。
 *
 * 9cm 总宽度对应约 1.286cm 的相邻间距。使用奇数权重后，
 * 一个权重单位约对应 0.643cm：
 * x1  x2  x3  x4  x5  x6  x7  x8
 * -7  -5  -3  -1  +1  +3  +5  +7
 */
static const int8_t SensorWeight[8] =
{
    -7, -5, -3, -1, 1, 3, 5, 7
};

/*
 * 转向方向总开关：
 * 物理左侧压黑时 A 减速、B 加速；物理右侧压黑时 A 加速、B 减速。
 */
#define LINE_STEER_SIGN             -1.0f

/*
 * 双段变增益与控制阈值配置（基于 W=19cm, L=19cm, R=50cm 实车标定）：
 * 1. 死区 0.3f：忽略直道小偏执与振动
 * 2. 小偏差阈值 1.5f：严格划分直线行驶与圆弧稳态外摆区
 * 3. 小偏差 Kp 1.5f：保障直道冷静行驶，消除小幅晃动
 * 4. 大偏差 Kp 2.38f：精确匹配 35cm/s 基础转速下 6.65cm/s 的稳态过弯向心差速
 * 5. 微分 Kd 1.2f：维持较低阻尼，不干预恒定弧线上的正常向心旋转
 */
#define LINE_ERROR_DEADBAND         0.3f
#define LINE_ERROR_SMALL_THRES      1.5f
#define LINE_KP_SMALL_CMS           1.5f
#define LINE_KP_LARGE_CMS           2.38f

#define LINE_KD_CMS                 1.2f
#define LINE_ERROR_FILTER_ALPHA     0.65f
#define LINE_D_FILTER_ALPHA         0.45f

/* === 新增宏：控制参数配置时的基准速度（35cm/s），用于全车速自适应缩放 === */
#define LINE_BASE_CALIB_SPEED_CMS   35.0f

/* 差速限幅与转速限幅 */
#define LINE_MAX_CORRECTION_CMS     12.0f
#define LINE_MIN_WHEEL_SPEED_CMS    10.0f
#define LINE_MAX_WHEEL_SPEED_CMS    50.0f

/* 丢线后最多搜索 80ms，执行降速、强差速拉回策略，超时后停车 */
#define LINE_LOST_SEARCH_TICKS      8U
#define LINE_LOST_BASE_SPEED_CMS    5.0f
#define LINE_LOST_TURN_CMS          15.0f

/* 连续 3 次 I2C 读取失败后停车 */
#define LINE_I2C_FAILURE_LIMIT      3U

/* 至少 4 路同时检测到黑线，连续 20ms 后认定为横向标志线 */
#define LINE_MARKER_SENSOR_COUNT    4U
#define LINE_MARKER_STABLE_TICKS    2U

static volatile LineTrackingState TrackingState = LINE_TRACK_STOPPED;

static float BaseSpeedCMS = 35.0f;
static float FilteredError = 0.0f;
static float PreviousFilteredError = 0.0f;
static float FilteredDerivative = 0.0f;
static float LastValidError = 0.0f;
static float CorrectionCMS = 0.0f;

static uint8_t LastRawData = 0xFFU;
static uint8_t ActiveSensorCount = 0U;
static uint8_t LostTicks = 0U;
static uint8_t I2CFailureCount = 0U;

static uint8_t MarkerStableTicks = 0U;
static uint8_t MarkerLatched = 0U;
static volatile uint8_t MarkerEvent = 0U;
static volatile uint32_t MarkerCount = 0U;

static float ClampFloat(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

static void ApplyWheelSpeed(float left_cms, float right_cms)
{
    left_cms = ClampFloat(left_cms,
                          LINE_MIN_WHEEL_SPEED_CMS,
                          LINE_MAX_WHEEL_SPEED_CMS);
    right_cms = ClampFloat(right_cms,
                           LINE_MIN_WHEEL_SPEED_CMS,
                           LINE_MAX_WHEEL_SPEED_CMS);

    Set_Target_Speed_CMS(left_cms, right_cms);
}

static void UpdateMarker(uint8_t active_count)
{
    if (active_count >= LINE_MARKER_SENSOR_COUNT)
    {
        if (MarkerStableTicks < LINE_MARKER_STABLE_TICKS)
        {
            MarkerStableTicks++;
        }

        if ((MarkerStableTicks >= LINE_MARKER_STABLE_TICKS)
         && (MarkerLatched == 0U))
        {
            MarkerLatched = 1U;
            MarkerEvent = 1U;
            MarkerCount++;
        }
    }
    else
    {
        MarkerStableTicks = 0U;

        /*
         * 滞回复位：探头触发数量降至 2 路及以下（恢复为正常细线循迹），
         * 才清空锁存标志，防止驶离横线时探头在 3~4 之间抖动导致重复计数。
         */
        if (active_count <= 2U)
        {
            MarkerLatched = 0U;
        }
    }
}

void LineTracking_Init(float base_speed_cms)
{
    BaseSpeedCMS = ClampFloat(base_speed_cms,
                              LINE_MIN_WHEEL_SPEED_CMS,
                              LINE_MAX_WHEEL_SPEED_CMS);

    FilteredError = 0.0f;
    PreviousFilteredError = 0.0f;
    FilteredDerivative = 0.0f;
    LastValidError = 0.0f;
    CorrectionCMS = 0.0f;

    LastRawData = 0xFFU;
    ActiveSensorCount = 0U;
    LostTicks = 0U;
    I2CFailureCount = 0U;

    MarkerStableTicks = 0U;
    MarkerLatched = 0U;
    MarkerEvent = 0U;
    MarkerCount = 0U;

    TrackingState = LINE_TRACK_STOPPED;
    Set_Target_Speed(0, 0);
}

void LineTracking_Start(void)
{
    FilteredError = 0.0f;
    PreviousFilteredError = 0.0f;
    FilteredDerivative = 0.0f;
    LastValidError = 0.0f;
    CorrectionCMS = 0.0f;
    LostTicks = 0U;
    I2CFailureCount = 0U;

    TrackingState = LINE_TRACK_RUNNING;
    Set_Target_Speed_CMS(BaseSpeedCMS, BaseSpeedCMS);
}

void LineTracking_SetBaseSpeed(float base_speed_cms)
{
    /*
     * 只修改基础速度，不清空循迹滤波、横线计数和运行状态。
     * 新速度会在下一次 LineTracking_Update() 中生效。
     */
    BaseSpeedCMS = ClampFloat(base_speed_cms,
                              LINE_MIN_WHEEL_SPEED_CMS,
                              LINE_MAX_WHEEL_SPEED_CMS);
}

void LineTracking_Stop(void)
{
    TrackingState = LINE_TRACK_STOPPED;
    CorrectionCMS = 0.0f;
    Set_Target_Speed(0, 0);
}

void LineTracking_Update(uint8_t raw_data)
{
    uint8_t i;
    uint8_t active_count = 0U;
    int16_t weight_sum = 0;
    float measured_error;
    float raw_derivative;
    float abs_error;
    float current_kp;
    float left_speed;
    float right_speed;
    float speed_ratio;

    if (TrackingState != LINE_TRACK_RUNNING)
    {
        return;
    }

    LastRawData = raw_data;
    I2CFailureCount = 0U;

    for (i = 0U; i < 8U; i++)
    {
        uint8_t bit_mask = (uint8_t)(0x80U >> i);

        if ((raw_data & bit_mask) == 0U)
        {
            weight_sum += SensorWeight[i];
            active_count++;
        }
    }

    ActiveSensorCount = active_count;
    UpdateMarker(active_count);

    if (active_count == 0U)
    {
        LostTicks++;

        if (LostTicks <= LINE_LOST_SEARCH_TICKS)
        {
            /*
             * 按最后一次看见黑线的方向找线：
             * 大幅降低推进速度、增大转向差速，加速原地偏转寻找。
             */
            if (LastValidError > 0.0f)
            {
                CorrectionCMS = LINE_LOST_TURN_CMS;
            }
            else if (LastValidError < 0.0f)
            {
                CorrectionCMS = -LINE_LOST_TURN_CMS;
            }
            else
            {
                CorrectionCMS = 0.0f;
            }

            ApplyWheelSpeed(LINE_LOST_BASE_SPEED_CMS + CorrectionCMS,
                            LINE_LOST_BASE_SPEED_CMS - CorrectionCMS);
        }
        else
        {
            TrackingState = LINE_TRACK_LINE_LOST;
            CorrectionCMS = 0.0f;
            Set_Target_Speed(0, 0);
        }

        return;
    }

    /*
     * 1. 触发横向标志线（发车/终止线）：
     *    保持基础转速直行穿过，并冻结当前滤波及微分历史状态，避免阶跃冲击。
     */
    if (active_count >= LINE_MARKER_SENSOR_COUNT)
    {
        CorrectionCMS = 0.0f;
        ApplyWheelSpeed(BaseSpeedCMS, BaseSpeedCMS);
        return;
    }

    /*
     * 2. 普通黑线识别：计算加权平均误差
     */
    measured_error = (float)weight_sum / (float)active_count;
    LastValidError = measured_error;

    /*
     * 3. 丢线恢复检查：
     *    如果上一帧处于丢线状态，重新连线的第一帧需强制同步历史误差，避免微分飞升。
     */
    if (LostTicks > 0U)
    {
        FilteredError = measured_error;
        PreviousFilteredError = measured_error;
        FilteredDerivative = 0.0f;
        LostTicks = 0U;
    }

    /*
     * 4. 误差一阶低通滤波
     */
    PreviousFilteredError = FilteredError;
    FilteredError +=
        LINE_ERROR_FILTER_ALPHA * (measured_error - FilteredError);

    /*
     * 5. 微分项一阶低通滤波：
     *    滤除各传感器状态跳跃瞬间引发的高频差速毛刺。
     */
    raw_derivative = FilteredError - PreviousFilteredError;
    FilteredDerivative +=
        LINE_D_FILTER_ALPHA * (raw_derivative - FilteredDerivative);

    /*
     * 6. 变增益分段控制与死区处理：
     *    小偏差时零 Kp 或弱 Kp 抑制摆动；偏离外侧时强 Kp 保障拉回。
     */
    abs_error = (FilteredError > 0.0f) ? FilteredError : -FilteredError;

    if (abs_error < LINE_ERROR_DEADBAND)
    {
        current_kp = 0.0f;
    }
    else if (abs_error <= LINE_ERROR_SMALL_THRES)
    {
        current_kp = LINE_KP_SMALL_CMS;
    }
    else
    {
        current_kp = LINE_KP_LARGE_CMS;
    }

    /*
     * 7. 计算差速修正输出（引入速度缩放机制）
     *    speed_ratio 为“当前设定速度 / 标称调参速度”，例如 20/35 = 0.571
     *    确保不同运行速度下，控制系统输出的转弯向心差速自动按正比放缩。
     */
    speed_ratio = BaseSpeedCMS / LINE_BASE_CALIB_SPEED_CMS;

    CorrectionCMS = LINE_STEER_SIGN
                  * speed_ratio
                  * (current_kp * FilteredError
                   + LINE_KD_CMS * FilteredDerivative);

    /* 差速限幅上限同样按照车速比例动态放缩 */
    CorrectionCMS = ClampFloat(CorrectionCMS,
                               -LINE_MAX_CORRECTION_CMS * speed_ratio,
                               LINE_MAX_CORRECTION_CMS * speed_ratio);

    /*
     * 正误差：黑线在右边，需要右转，因此左轮加速、右轮减速。
     * 负误差：黑线在左边，需要左转，因此左轮减速、右轮加速。
     */
    left_speed = BaseSpeedCMS + CorrectionCMS;
    right_speed = BaseSpeedCMS - CorrectionCMS;
    ApplyWheelSpeed(left_speed, right_speed);
}

void LineTracking_OnI2CFailure(void)
{
    if (TrackingState != LINE_TRACK_RUNNING)
    {
        return;
    }

    if (I2CFailureCount < LINE_I2C_FAILURE_LIMIT)
    {
        I2CFailureCount++;
    }

    if (I2CFailureCount >= LINE_I2C_FAILURE_LIMIT)
    {
        TrackingState = LINE_TRACK_I2C_FAULT;
        CorrectionCMS = 0.0f;
        Set_Target_Speed(0, 0);
    }
}

LineTrackingState LineTracking_GetState(void)
{
    return TrackingState;
}

float LineTracking_GetError(void)
{
    return FilteredError;
}

float LineTracking_GetCorrectionCMS(void)
{
    return CorrectionCMS;
}

uint8_t LineTracking_GetRawData(void)
{
    return LastRawData;
}

uint8_t LineTracking_GetActiveCount(void)
{
    return ActiveSensorCount;
}

uint8_t LineTracking_GetMarkerEvent(void)
{
    uint8_t event = MarkerEvent;
    MarkerEvent = 0U;
    return event;
}

uint32_t LineTracking_GetMarkerCount(void)
{
    return MarkerCount;
}
