#include "pulse_stepper.h"
#include "tim.h"

#define PULSE_TIMER_CLOCK_HZ       1000000UL /* TIM3/TIM4: 64MHz/(63+1)=1MHz */
#define PULSE_MOTOR_FULL_STEPS     200UL     /* 1.8度步进电机每圈200整步 */
#define PULSE_MICROSTEPS           16UL      /* 驱动板细分设置为16 */
#define PULSE_STEPS_PER_REV        (PULSE_MOTOR_FULL_STEPS * PULSE_MICROSTEPS)
#define PULSE_MIN_PPS              20UL      /* 低于该频率时直接停，避免脉冲很稀疏 */
#define PULSE_MAX_PPS              8000UL
#define PULSE_DIR_SETUP_MS         1U        /* 换向后留1ms方向建立时间 */

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  uint8_t running;
  uint8_t active_dir;
  uint8_t pending_valid;
  uint8_t pending_dir;
  uint32_t pending_pps;
  uint8_t dir_delay_ms;
} PulseAxis_t;

static PulseAxis_t g_x_axis;
static PulseAxis_t g_y_axis;

static uint32_t Pulse_RpmToPps(uint16_t rpm)
{
  uint32_t pps = ((uint32_t)rpm * PULSE_STEPS_PER_REV + 30UL) / 60UL;

  if (pps < PULSE_MIN_PPS)
  {
    return 0UL;
  }
  if (pps > PULSE_MAX_PPS)
  {
    return PULSE_MAX_PPS;
  }
  return pps;
}

static void Pulse_StopAxis(PulseAxis_t *axis)
{
  (void)HAL_TIM_PWM_Stop(axis->htim, axis->channel);
  __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, 0U);
  axis->running = 0U;
  axis->pending_valid = 0U;
  axis->dir_delay_ms = 0U;
}

static HAL_StatusTypeDef Pulse_StartAxis(PulseAxis_t *axis, uint8_t dir, uint32_t pps)
{
  uint32_t period;
  uint8_t was_running = axis->running;

  if (pps == 0UL)
  {
    Pulse_StopAxis(axis);
    return HAL_OK;
  }

  period = (PULSE_TIMER_CLOCK_HZ / pps);
  if (period < 2UL)
  {
    period = 2UL;
  }
  if (period > 65536UL)
  {
    period = 65536UL;
  }

  HAL_GPIO_WritePin(axis->dir_port, axis->dir_pin, (dir == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  axis->active_dir = dir;

  if (was_running != 0U)
  {
    __HAL_TIM_DISABLE(axis->htim);
  }
  __HAL_TIM_SET_AUTORELOAD(axis->htim, period - 1UL);
  __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, period / 2UL);
  __HAL_TIM_SET_COUNTER(axis->htim, 0U);
  if (was_running != 0U)
  {
    __HAL_TIM_ENABLE(axis->htim);
    return HAL_OK;
  }

  axis->running = 1U;
  return HAL_TIM_PWM_Start(axis->htim, axis->channel);
}

static HAL_StatusTypeDef Pulse_CommandAxis(PulseAxis_t *axis, uint8_t dir, uint16_t rpm)
{
  uint32_t pps = Pulse_RpmToPps(rpm);

  if (pps == 0UL)
  {
    Pulse_StopAxis(axis);
    return HAL_OK;
  }

  if ((axis->running != 0U) && (dir != axis->active_dir))
  {
    Pulse_StopAxis(axis);
    HAL_GPIO_WritePin(axis->dir_port, axis->dir_pin, (dir == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    axis->pending_valid = 1U;
    axis->pending_dir = dir;
    axis->pending_pps = pps;
    axis->dir_delay_ms = PULSE_DIR_SETUP_MS;
    return HAL_OK;
  }

  return Pulse_StartAxis(axis, dir, pps);
}

static void Pulse_ProcessAxis1ms(PulseAxis_t *axis)
{
  if (axis->pending_valid == 0U)
  {
    return;
  }

  if (axis->dir_delay_ms > 0U)
  {
    axis->dir_delay_ms--;
    return;
  }

  axis->pending_valid = 0U;
  (void)Pulse_StartAxis(axis, axis->pending_dir, axis->pending_pps);
}

HAL_StatusTypeDef PulseStepper_Init(void)
{
  g_x_axis.htim = &htim3;
  g_x_axis.channel = TIM_CHANNEL_1;
  g_x_axis.dir_port = X_DIR_GPIO_Port;
  g_x_axis.dir_pin = X_DIR_Pin;

  g_y_axis.htim = &htim4;
  g_y_axis.channel = TIM_CHANNEL_4;
  g_y_axis.dir_port = Y_DIR_GPIO_Port;
  g_y_axis.dir_pin = Y_DIR_Pin;

  PulseStepper_StopAll();
  return HAL_OK;
}

void PulseStepper_Process1ms(void)
{
  Pulse_ProcessAxis1ms(&g_x_axis);
  Pulse_ProcessAxis1ms(&g_y_axis);
}

void PulseStepper_StopX(void)
{
  Pulse_StopAxis(&g_x_axis);
}

void PulseStepper_StopY(void)
{
  Pulse_StopAxis(&g_y_axis);
}

void PulseStepper_StopAll(void)
{
  Pulse_StopAxis(&g_x_axis);
  Pulse_StopAxis(&g_y_axis);
}

HAL_StatusTypeDef PulseStepper_RunXSpeed(uint8_t dir, uint16_t speed_rpm)
{
  return Pulse_CommandAxis(&g_x_axis, dir, speed_rpm);
}

HAL_StatusTypeDef PulseStepper_RunYSpeed(uint8_t dir, uint16_t speed_rpm)
{
  return Pulse_CommandAxis(&g_y_axis, dir, speed_rpm);
}
