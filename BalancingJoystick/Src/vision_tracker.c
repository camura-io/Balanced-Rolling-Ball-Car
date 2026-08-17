#include "vision_tracker.h"
#include "emm_stepper.h"
#include "imu901.h"
#include "pulse_stepper.h"
#include <stdlib.h>
#include <string.h>

#define VISION_LINE_MAX              64U
#define VISION_KEY_DEBOUNCE_MS       20U
#define VISION_ENABLE_SETTLE_MS      80U
#define VISION_OUTER_PERIOD_MS       50U  /* 外环跟随K230发送周期，把像素误差换成角度目标。 */
#define VISION_INNER_PERIOD_MS       20U  /* X轴内环跟随MS901M 50Hz yaw，把角度误差换成电机速度。 */
#define VISION_TARGET_TIMEOUT_MS     300U
#define VISION_IMU_TIMEOUT_MS        160U

#define VISION_X_DEADBAND_PX         18.0f /* 光斑进入该像素范围，外环不再给新角度偏置。 */
#define VISION_Y_DEADBAND_PX         18.0f
#define VISION_OUTER_X_MAX_DEG       8.0f  /* 单次外环允许给内环的最大yaw偏置。 */
#define VISION_OUTER_X_KP_DEG_PER_PX 0.018f
#define VISION_OUTER_X_KI            0.0f
#define VISION_OUTER_X_KD            0.0f

#define VISION_ANGLE_DEADBAND_DEG    0.28f /* X轴yaw内环角度死区，小于该值直接停车。 */
#define VISION_INNER_X_MAX_RPM       65U
#define VISION_INNER_MIN_RPM         2U
#define VISION_INNER_FINE_DEG        1.2f
#define VISION_INNER_FINE_MAX_RPM    8U
#define VISION_INNER_ACC            6U
#define VISION_RPM_STEP              2U    /* 速度分档，减少驱动器反复重规划。 */
#define VISION_INNER_X_KP_RPM_DEG    7.0f
#define VISION_INNER_X_KI            0.0f
#define VISION_INNER_X_KD            0.20f

#define VISION_Y_MAX_RPM             50U   /* Y轴不进陀螺仪，直接由视觉dy控制。 */
#define VISION_Y_NEAR_PX             60.0f
#define VISION_Y_FINE_PX             36.0f
#define VISION_Y_NEAR_MAX_RPM        18U
#define VISION_Y_FINE_MAX_RPM        6U
#define VISION_Y_KP_RPM_PER_PX       0.11f
#define VISION_Y_KI_RPM_PER_PXS      0.0f
#define VISION_Y_KD_RPM_PER_PXS      0.0f
#define VISION_PID_I_LIMIT           200.0f

#define VISION_X_POSITIVE_DIR        0U    /* yaw目标-当前yaw为正时X轴方向，反了改成1。 */
#define VISION_Y_POSITIVE_DIR        0U    /* dy为正时Y轴方向，反了改成1。 */
#define VISION_X_SELFTEST_MS         1000U /* K2启动后先给X轴1秒固定脉冲，隔离检查STEP/DIR接线。 */
#define VISION_X_SELFTEST_RPM        12U
#define VISION_X_SELFTEST_DIR        0U

typedef enum
{
  VISION_IDLE = 0,
  VISION_ENABLE_MOTORS,
  VISION_ACTIVE
} VisionState_t;

typedef struct
{
  int16_t cx;
  int16_t cy;
  int16_t dx;
  int16_t dy;
  int16_t w;
  int16_t h;
  uint8_t valid;
  uint32_t tick;
} VisionTarget_t;

typedef struct
{
  float integral;
  float last_error;
  uint8_t ready;
} VisionPid_t;

static UART_HandleTypeDef *g_vision_uart;
static uint8_t g_uart_rx_byte;
static char g_rx_line[VISION_LINE_MAX];
static char g_parse_line[VISION_LINE_MAX];
static volatile uint8_t g_rx_index;
static volatile uint8_t g_line_ready;
static VisionTarget_t g_target;
static VisionState_t g_state;
static uint16_t g_enable_delay_ms;
static uint16_t g_outer_tick_ms;
static uint16_t g_inner_tick_ms;
static uint8_t g_x_stopped;
static uint8_t g_y_stopped;
static uint8_t g_last_x_dir;
static uint8_t g_last_y_dir;
static uint16_t g_last_x_rpm;
static uint16_t g_last_y_rpm;
static uint8_t g_had_fresh_target;
static uint16_t g_x_selftest_ms;
static float g_target_yaw;
static VisionPid_t g_outer_x_pid;
static VisionPid_t g_inner_x_pid;
static VisionPid_t g_y_pixel_pid;

static int16_t Vision_ParseInt(char **cursor)
{
  long value;

  if (**cursor == ',')
  {
    (*cursor)++;
  }

  value = strtol(*cursor, cursor, 10);
  if (value > 32767L)
  {
    return 32767;
  }
  if (value < -32768L)
  {
    return -32768;
  }
  return (int16_t)value;
}

static float Vision_AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float Vision_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float Vision_WrapAngle(float angle)
{
  while (angle > 180.0f)
  {
    angle -= 360.0f;
  }
  while (angle < -180.0f)
  {
    angle += 360.0f;
  }
  return angle;
}

static void Vision_ResetPid(VisionPid_t *pid)
{
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->ready = 0U;
}

static float Vision_PidStep(VisionPid_t *pid, float error, float dt_s, float kp, float ki, float kd)
{
  float derivative = 0.0f;

  if (pid->ready != 0U)
  {
    derivative = (error - pid->last_error) / dt_s;
  }
  else
  {
    pid->ready = 1U;
  }

  pid->integral += error * dt_s;
  pid->integral = Vision_ClampFloat(pid->integral, -VISION_PID_I_LIMIT, VISION_PID_I_LIMIT);
  pid->last_error = error;

  return (kp * error) + (ki * pid->integral) + (kd * derivative);
}

static uint16_t Vision_LimitRpm(float rpm, uint16_t max_rpm)
{
  uint16_t out;

  rpm = Vision_AbsFloat(rpm);
  if (rpm < (float)VISION_INNER_MIN_RPM)
  {
    out = VISION_INNER_MIN_RPM;
  }
  else if (rpm > (float)max_rpm)
  {
    out = max_rpm;
  }
  else
  {
    out = (uint16_t)rpm;
  }

  out = (uint16_t)(((out + (VISION_RPM_STEP / 2U)) / VISION_RPM_STEP) * VISION_RPM_STEP);
  if (out < VISION_INNER_MIN_RPM)
  {
    return VISION_INNER_MIN_RPM;
  }
  if (out > max_rpm)
  {
    return max_rpm;
  }
  return out;
}

static void Vision_StopX(void)
{
  if (g_x_stopped == 0U)
  {
    PulseStepper_StopX();
  }
  g_x_stopped = 1U;
  g_last_x_rpm = 0U;
  Vision_ResetPid(&g_inner_x_pid);
}

static void Vision_StopY(void)
{
  if (g_y_stopped == 0U)
  {
    PulseStepper_StopY();
  }
  g_y_stopped = 1U;
  g_last_y_rpm = 0U;
  Vision_ResetPid(&g_y_pixel_pid);
}

static void Vision_StopAll(void)
{
  Vision_StopX();
  Vision_StopY();
}

static void Vision_ResetMotionCache(void)
{
  g_x_stopped = 1U;
  g_y_stopped = 1U;
  g_last_x_dir = 0U;
  g_last_y_dir = 0U;
  g_last_x_rpm = 0U;
  g_last_y_rpm = 0U;
  g_had_fresh_target = 0U;
  g_x_selftest_ms = 0U;
  Vision_ResetPid(&g_outer_x_pid);
  Vision_ResetPid(&g_inner_x_pid);
  Vision_ResetPid(&g_y_pixel_pid);
}

static void Vision_StopAndIdle(void)
{
  Vision_StopAll();
  g_state = VISION_IDLE;
  g_enable_delay_ms = 0U;
  g_outer_tick_ms = 0U;
  g_inner_tick_ms = 0U;
  g_had_fresh_target = 0U;
  g_x_selftest_ms = 0U;
  Vision_ResetPid(&g_outer_x_pid);
  Vision_ResetPid(&g_y_pixel_pid);
}

static void Vision_ParseLine(const char *line)
{
  char *cursor;

  if (line[0] == 'N')
  {
    g_target.valid = 0U;
    g_target.tick = HAL_GetTick();
    return;
  }

  if (line[0] != 'T')
  {
    return;
  }

  strncpy(g_parse_line, line, sizeof(g_parse_line) - 1U);
  g_parse_line[sizeof(g_parse_line) - 1U] = '\0';
  cursor = &g_parse_line[1];

  g_target.cx = Vision_ParseInt(&cursor);
  g_target.cy = Vision_ParseInt(&cursor);
  g_target.dx = Vision_ParseInt(&cursor);
  g_target.dy = Vision_ParseInt(&cursor);
  g_target.w = Vision_ParseInt(&cursor);
  g_target.h = Vision_ParseInt(&cursor);
  g_target.valid = 1U;
  g_target.tick = HAL_GetTick();
}

static void Vision_ProcessRxLine(void)
{
  char local_line[VISION_LINE_MAX];

  if (g_line_ready == 0U)
  {
    return;
  }

  __disable_irq();
  strncpy(local_line, g_rx_line, sizeof(local_line) - 1U);
  local_line[sizeof(local_line) - 1U] = '\0';
  g_line_ready = 0U;
  __enable_irq();

  Vision_ParseLine(local_line);
}

static uint8_t Vision_ReadK2Press(void)
{
  static uint8_t stable_level = GPIO_PIN_SET;
  static uint8_t last_sample = GPIO_PIN_SET;
  static uint16_t debounce_ms;
  uint8_t sample = (uint8_t)HAL_GPIO_ReadPin(KEY_K2_GPIO_Port, KEY_K2_Pin);

  if (sample != last_sample)
  {
    last_sample = sample;
    debounce_ms = 0U;
    return 0U;
  }

  if (debounce_ms < VISION_KEY_DEBOUNCE_MS)
  {
    debounce_ms++;
    return 0U;
  }

  if (sample != stable_level)
  {
    stable_level = sample;
    return (stable_level == GPIO_PIN_RESET) ? 1U : 0U;
  }

  return 0U;
}

static uint8_t Vision_GetFreshImu(IMU901_Data_t *imu)
{
  if (IMU901_HasFreshAngle(VISION_IMU_TIMEOUT_MS) == 0U)
  {
    return 0U;
  }

  IMU901_GetData(imu);
  return 1U;
}

static void Vision_Start(void)
{
  IMU901_Data_t imu;

  if ((EMM_Stepper_IsBusy() != 0U) || (g_state != VISION_IDLE))
  {
    return;
  }
  if (Vision_GetFreshImu(&imu) == 0U)
  {
    return;
  }

  (void)EMM_Stepper_EnableX(1U);
  (void)EMM_Stepper_EnableY(1U);
  Vision_ResetMotionCache();
  g_target_yaw = imu.yaw;
  g_state = VISION_ENABLE_MOTORS;
  g_enable_delay_ms = VISION_ENABLE_SETTLE_MS;
  g_outer_tick_ms = 0U;
  g_inner_tick_ms = 0U;
}

static uint8_t Vision_TargetFresh(void)
{
  return ((g_target.valid != 0U) && ((HAL_GetTick() - g_target.tick) <= VISION_TARGET_TIMEOUT_MS)) ? 1U : 0U;
}

static void Vision_UpdateOuterLoop(const IMU901_Data_t *imu)
{
  float yaw_offset;

  if (Vision_TargetFresh() == 0U)
  {
    if (g_had_fresh_target != 0U)
    {
      /* 目标丢失瞬间冻结当前yaw，Y轴直接停车，避免继续追上一帧旧视觉数据。 */
      g_target_yaw = imu->yaw;
      g_had_fresh_target = 0U;
    }
    Vision_ResetPid(&g_outer_x_pid);
    Vision_StopY();
    return;
  }

  g_had_fresh_target = 1U;

  if (Vision_AbsFloat((float)g_target.dx) <= VISION_X_DEADBAND_PX)
  {
    g_target_yaw = imu->yaw;
    Vision_ResetPid(&g_outer_x_pid);
  }
  else
  {
    yaw_offset = Vision_PidStep(&g_outer_x_pid, (float)g_target.dx,
                                (float)VISION_OUTER_PERIOD_MS / 1000.0f,
                                VISION_OUTER_X_KP_DEG_PER_PX,
                                VISION_OUTER_X_KI,
                                VISION_OUTER_X_KD);
    yaw_offset = Vision_ClampFloat(yaw_offset, -VISION_OUTER_X_MAX_DEG, VISION_OUTER_X_MAX_DEG);
    g_target_yaw = Vision_WrapAngle(imu->yaw + yaw_offset);
  }
}

static void Vision_RunAxisXByAngle(float angle_error)
{
  float abs_error = Vision_AbsFloat(angle_error);
  float cmd;
  uint16_t max_rpm;
  uint16_t rpm;
  uint8_t dir;

  if (abs_error <= VISION_ANGLE_DEADBAND_DEG)
  {
    Vision_StopX();
    return;
  }

  cmd = Vision_PidStep(&g_inner_x_pid, angle_error,
                       (float)VISION_INNER_PERIOD_MS / 1000.0f,
                       VISION_INNER_X_KP_RPM_DEG,
                       VISION_INNER_X_KI,
                       VISION_INNER_X_KD);
  max_rpm = (abs_error <= VISION_INNER_FINE_DEG) ? VISION_INNER_FINE_MAX_RPM : VISION_INNER_X_MAX_RPM;
  rpm = Vision_LimitRpm(cmd, max_rpm);
  dir = (cmd >= 0.0f) ? VISION_X_POSITIVE_DIR : (uint8_t)(1U - VISION_X_POSITIVE_DIR);

  if ((g_x_stopped != 0U) || (dir != g_last_x_dir) || (rpm != g_last_x_rpm))
  {
    (void)PulseStepper_RunXSpeed(dir, rpm);
    g_x_stopped = 0U;
    g_last_x_dir = dir;
    g_last_x_rpm = rpm;
  }
}

static void Vision_RunAxisYByPixel(int16_t dy)
{
  float abs_error = Vision_AbsFloat((float)dy);
  float cmd;
  uint16_t max_rpm;
  uint16_t rpm;
  uint8_t dir;

  if (abs_error <= VISION_Y_DEADBAND_PX)
  {
    Vision_StopY();
    return;
  }

  cmd = Vision_PidStep(&g_y_pixel_pid, (float)dy,
                       (float)VISION_OUTER_PERIOD_MS / 1000.0f,
                       VISION_Y_KP_RPM_PER_PX,
                       VISION_Y_KI_RPM_PER_PXS,
                       VISION_Y_KD_RPM_PER_PXS);

  if (abs_error <= VISION_Y_FINE_PX)
  {
    max_rpm = VISION_Y_FINE_MAX_RPM;
  }
  else if (abs_error <= VISION_Y_NEAR_PX)
  {
    max_rpm = VISION_Y_NEAR_MAX_RPM;
  }
  else
  {
    max_rpm = VISION_Y_MAX_RPM;
  }

  rpm = Vision_LimitRpm(cmd, max_rpm);
  dir = (cmd >= 0.0f) ? VISION_Y_POSITIVE_DIR : (uint8_t)(1U - VISION_Y_POSITIVE_DIR);

  if ((g_y_stopped != 0U) || (dir != g_last_y_dir) || (rpm != g_last_y_rpm))
  {
    (void)PulseStepper_RunYSpeed(dir, rpm);
    g_y_stopped = 0U;
    g_last_y_dir = dir;
    g_last_y_rpm = rpm;
  }
}

static void Vision_UpdateInnerLoop(const IMU901_Data_t *imu)
{
  Vision_RunAxisXByAngle(Vision_WrapAngle(g_target_yaw - imu->yaw));
}

static void Vision_UpdateDirectYLoop(void)
{
  if (Vision_TargetFresh() == 0U)
  {
    Vision_StopY();
    return;
  }

  Vision_RunAxisYByPixel(g_target.dy);
}

HAL_StatusTypeDef VisionTracker_Init(UART_HandleTypeDef *huart)
{
  g_vision_uart = huart;
  g_rx_index = 0U;
  g_line_ready = 0U;
  memset(&g_target, 0, sizeof(g_target));
  g_state = VISION_IDLE;
  g_enable_delay_ms = 0U;
  g_outer_tick_ms = 0U;
  g_inner_tick_ms = 0U;
  g_target_yaw = 0.0f;
  Vision_ResetMotionCache();

  return HAL_UART_Receive_IT(g_vision_uart, &g_uart_rx_byte, 1U);
}

void VisionTracker_Process1ms(void)
{
  IMU901_Data_t imu;

  Vision_ProcessRxLine();

  if (EMM_Stepper_IsBusy() != 0U)
  {
    if (g_state != VISION_IDLE)
    {
      Vision_StopAndIdle();
    }
    return;
  }

  if (Vision_ReadK2Press() != 0U)
  {
    if (g_state == VISION_IDLE)
    {
      Vision_Start();
    }
    else
    {
      Vision_StopAndIdle();
    }
  }

  if (g_state == VISION_ENABLE_MOTORS)
  {
    if (g_enable_delay_ms > 0U)
    {
      g_enable_delay_ms--;
      return;
    }
    g_state = VISION_ACTIVE;
    g_x_selftest_ms = VISION_X_SELFTEST_MS;
  }

  if (g_state != VISION_ACTIVE)
  {
    return;
  }

  if (Vision_GetFreshImu(&imu) == 0U)
  {
    Vision_StopAndIdle();
    return;
  }

  if (g_x_selftest_ms > 0U)
  {
    if (g_x_selftest_ms == VISION_X_SELFTEST_MS)
    {
      (void)PulseStepper_RunXSpeed(VISION_X_SELFTEST_DIR, VISION_X_SELFTEST_RPM);
      g_x_stopped = 0U;
      g_last_x_dir = VISION_X_SELFTEST_DIR;
      g_last_x_rpm = VISION_X_SELFTEST_RPM;
      Vision_StopY();
    }

    g_x_selftest_ms--;
    if (g_x_selftest_ms == 0U)
    {
      Vision_StopX();
      g_target_yaw = imu.yaw;
      g_outer_tick_ms = 0U;
      g_inner_tick_ms = 0U;
    }
    return;
  }

  g_outer_tick_ms++;
  if (g_outer_tick_ms >= VISION_OUTER_PERIOD_MS)
  {
    g_outer_tick_ms = 0U;
    Vision_UpdateOuterLoop(&imu);
    Vision_UpdateDirectYLoop();
  }

  g_inner_tick_ms++;
  if (g_inner_tick_ms >= VISION_INNER_PERIOD_MS)
  {
    g_inner_tick_ms = 0U;
    Vision_UpdateInnerLoop(&imu);
  }
}

uint8_t VisionTracker_IsActive(void)
{
  return (g_state == VISION_ACTIVE) ? 1U : 0U;
}

uint8_t VisionTracker_HasFreshTarget(void)
{
  return Vision_TargetFresh();
}

void VisionTracker_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((g_vision_uart != NULL) && (huart->Instance == g_vision_uart->Instance))
  {
    if ((g_uart_rx_byte == '\n') || (g_uart_rx_byte == '\r'))
    {
      if ((g_rx_index > 0U) && (g_line_ready == 0U))
      {
        g_rx_line[g_rx_index] = '\0';
        g_line_ready = 1U;
      }
      g_rx_index = 0U;
    }
    else if (g_rx_index < (VISION_LINE_MAX - 1U))
    {
      g_rx_line[g_rx_index++] = (char)g_uart_rx_byte;
    }
    else
    {
      g_rx_index = 0U;
    }

    (void)HAL_UART_Receive_IT(g_vision_uart, &g_uart_rx_byte, 1U);
  }
}

void VisionTracker_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((g_vision_uart != NULL) && (huart->Instance == g_vision_uart->Instance))
  {
    __HAL_UART_CLEAR_OREFLAG(g_vision_uart);
    g_rx_index = 0U;
    (void)HAL_UART_Receive_IT(g_vision_uart, &g_uart_rx_byte, 1U);
  }
}
