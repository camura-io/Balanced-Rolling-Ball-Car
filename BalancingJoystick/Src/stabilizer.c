#include "stabilizer.h"
#include "emm_stepper.h"
#include "imu901.h"

#define STAB_KEY_DEBOUNCE_MS       20U
#define STAB_LOOP_MS               20U
#define STAB_IMU_TIMEOUT_MS        300U
#define STAB_X_ENABLE_SETTLE_MS    60U
#define STAB_X_ACC                 10U
#define STAB_X_MAX_RPM             60U
#define STAB_X_MIN_RPM             6U
#define STAB_RPM_STEP              10U
#define STAB_DEADBAND_DEG          1.0f
#define STAB_KP_RPM_PER_DEG        3.5f
#define STAB_POSITIVE_ERROR_DIR    0U  /* yaw正误差对应的X轴方向，现场已确认需要反向。 */

typedef enum
{
  STAB_IDLE = 0,
  STAB_ENABLE_X,
  STAB_ACTIVE
} Stabilizer_State_t;

static Stabilizer_State_t g_stab_state;
static float g_target_yaw;
static uint16_t g_enable_delay_ms;
static uint16_t g_loop_tick_ms;
static uint8_t g_x_stopped;
static uint8_t g_last_dir;
static uint16_t g_last_rpm;

static float Stabilizer_WrapAngle(float angle)
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

static float Stabilizer_AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static uint8_t Stabilizer_ReadK2Press(void)
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

  if (debounce_ms < STAB_KEY_DEBOUNCE_MS)
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

static void Stabilizer_StopAndIdle(void)
{
  if (g_x_stopped == 0U)
  {
    (void)EMM_Stepper_StopX();
  }

  g_stab_state = STAB_IDLE;
  g_enable_delay_ms = 0U;
  g_loop_tick_ms = 0U;
  g_x_stopped = 1U;
  g_last_rpm = 0U;
}

static void Stabilizer_StartFromCurrentYaw(void)
{
  IMU901_Data_t imu;

  if ((IMU901_HasFreshAngle(STAB_IMU_TIMEOUT_MS) == 0U) || (EMM_Stepper_IsBusy() != 0U))
  {
    return;
  }

  IMU901_GetData(&imu);
  g_target_yaw = imu.yaw;

  /* K2按下时锁定当前yaw，先使能X轴，再进入水平稳定。 */
  (void)EMM_Stepper_EnableX(1U);
  g_stab_state = STAB_ENABLE_X;
  g_enable_delay_ms = STAB_X_ENABLE_SETTLE_MS;
  g_loop_tick_ms = 0U;
  g_x_stopped = 1U;
  g_last_rpm = 0U;
}

static void Stabilizer_ControlX(void)
{
  IMU901_Data_t imu;
  float error;
  float abs_error;
  uint16_t rpm;
  uint8_t dir;

  if (IMU901_HasFreshAngle(STAB_IMU_TIMEOUT_MS) == 0U)
  {
    Stabilizer_StopAndIdle();
    return;
  }

  IMU901_GetData(&imu);
  error = Stabilizer_WrapAngle(g_target_yaw - imu.yaw);
  abs_error = Stabilizer_AbsFloat(error);

  if (abs_error <= STAB_DEADBAND_DEG)
  {
    if (g_x_stopped == 0U)
    {
      (void)EMM_Stepper_StopX();
      g_x_stopped = 1U;
      g_last_rpm = 0U;
    }
    return;
  }

  rpm = (uint16_t)(abs_error * STAB_KP_RPM_PER_DEG);
  if (rpm < STAB_X_MIN_RPM)
  {
    rpm = STAB_X_MIN_RPM;
  }
  else if (rpm > STAB_X_MAX_RPM)
  {
    rpm = STAB_X_MAX_RPM;
  }

  /* 速度按10RPM分档，减少速度命令重发，避免驱动器反复重规划导致一段一段。 */
  rpm = (uint16_t)(((rpm + (STAB_RPM_STEP / 2U)) / STAB_RPM_STEP) * STAB_RPM_STEP);
  if (rpm < STAB_X_MIN_RPM)
  {
    rpm = STAB_X_MIN_RPM;
  }
  else if (rpm > STAB_X_MAX_RPM)
  {
    rpm = STAB_X_MAX_RPM;
  }

  dir = (error >= 0.0f) ? STAB_POSITIVE_ERROR_DIR : (uint8_t)(1U - STAB_POSITIVE_ERROR_DIR);
  if ((g_x_stopped != 0U) || (dir != g_last_dir) || (rpm != g_last_rpm))
  {
    (void)EMM_Stepper_RunXSpeed(dir, rpm, STAB_X_ACC);
    g_x_stopped = 0U;
    g_last_dir = dir;
    g_last_rpm = rpm;
  }
}

void Stabilizer_Init(void)
{
  g_stab_state = STAB_IDLE;
  g_target_yaw = 0.0f;
  g_enable_delay_ms = 0U;
  g_loop_tick_ms = 0U;
  g_x_stopped = 1U;
  g_last_dir = 0U;
  g_last_rpm = 0U;
}

void Stabilizer_Process1ms(void)
{
  if (EMM_Stepper_IsBusy() != 0U)
  {
    if (g_stab_state != STAB_IDLE)
    {
      Stabilizer_StopAndIdle();
    }
    return;
  }

  if (Stabilizer_ReadK2Press() != 0U)
  {
    if (g_stab_state == STAB_IDLE)
    {
      Stabilizer_StartFromCurrentYaw();
    }
    else
    {
      /* 稳定模式中再次按K2，立即停X轴并退出，防止旧速度命令持续运行。 */
      Stabilizer_StopAndIdle();
    }
  }

  if (g_stab_state == STAB_ENABLE_X)
  {
    if (g_enable_delay_ms > 0U)
    {
      g_enable_delay_ms--;
      return;
    }

    g_stab_state = STAB_ACTIVE;
  }

  if (g_stab_state != STAB_ACTIVE)
  {
    return;
  }

  g_loop_tick_ms++;
  if (g_loop_tick_ms >= STAB_LOOP_MS)
  {
    g_loop_tick_ms = 0U;
    Stabilizer_ControlX();
  }
}

uint8_t Stabilizer_IsActive(void)
{
  return (g_stab_state == STAB_ACTIVE) ? 1U : 0U;
}
