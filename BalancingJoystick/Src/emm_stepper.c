#include "emm_stepper.h"
#include "pulse_stepper.h"

#define EMM_ADDR_DEFAULT       0x01U
#define EMM_CHECKSUM_FIXED     0x6BU
#define EMM_SYNC_DISABLE       0x00U
#define EMM_KEY_DEBOUNCE_MS    20U
#define EMM_CMD_GAP_MS         20U
#define EMM_ENABLE_SETTLE_MS   100U
#define EMM_UART_TIMEOUT_MS    3U
#define EMM_DIR_CW             0x00U
#define EMM_DIR_CCW            0x01U
#define MKS_FRAME_HEAD         0xFAU
#define MKS_ADDR_DEFAULT       0x01U
#define MKS_MODE_CR_VFOC       0x02U /* MKS脉冲接口FOC模式，用STEP/DIR接PID脉冲。 */
#define MKS_EN_LEVEL_LOW       0x00U /* MKS En=L：低电平使能，高电平释放电机。 */
#define MKS_HOME_NEAR_MODE     0x02U
#define MKS_HOME_KEEP_ZERO     0x02U
#define MKS_HOME_SPEED         0x02U
#define MKS_HOME_DIR_CW        0x00U
#define MKS_GO_HOME_ORIGIN     0x00U

typedef enum
{
  EMM_SEQ_IDLE = 0,
  EMM_SEQ_ENABLE_Y,
  EMM_SEQ_ENABLE_X,
  EMM_SEQ_HOME_Y,
  EMM_SEQ_HOME_X,
  EMM_SEQ_RESTORE_X_PULSE_MODE
} EMM_SequenceState_t;

static UART_HandleTypeDef *g_y_uart;
static UART_HandleTypeDef *g_x_uart;
static EMM_SequenceState_t g_seq_state;
static uint16_t g_seq_delay_ms;

static HAL_StatusTypeDef EMM_SendEnable(UART_HandleTypeDef *huart, uint8_t enable)
{
  /* F3：使能信号控制，01使能，00失能。 */
  uint8_t cmd[] = {EMM_ADDR_DEFAULT, 0xF3U, 0xABU, enable, EMM_SYNC_DISABLE, EMM_CHECKSUM_FIXED};
  return HAL_UART_Transmit(huart, cmd, sizeof(cmd), EMM_UART_TIMEOUT_MS);
}

static HAL_StatusTypeDef EMM_SendNearestHome(UART_HandleTypeDef *huart)
{
  /* 9A：触发回零，00为单圈就近回零。 */
  uint8_t cmd[] = {EMM_ADDR_DEFAULT, 0x9AU, 0x00U, EMM_SYNC_DISABLE, EMM_CHECKSUM_FIXED};
  return HAL_UART_Transmit(huart, cmd, sizeof(cmd), EMM_UART_TIMEOUT_MS);
}

static HAL_StatusTypeDef EMM_SendSpeedMode(UART_HandleTypeDef *huart, uint8_t dir, uint16_t speed_rpm, uint8_t acc)
{
  /* F6：速度模式，速度单位RPM，高字节在前；sync=0立即执行。 */
  uint8_t cmd[] = {
    EMM_ADDR_DEFAULT,
    0xF6U,
    (dir == 0U) ? EMM_DIR_CW : EMM_DIR_CCW,
    (uint8_t)(speed_rpm >> 8),
    (uint8_t)(speed_rpm & 0xFFU),
    acc,
    EMM_SYNC_DISABLE,
    EMM_CHECKSUM_FIXED
  };

  return HAL_UART_Transmit(huart, cmd, sizeof(cmd), EMM_UART_TIMEOUT_MS);
}

static HAL_StatusTypeDef EMM_SendStop(UART_HandleTypeDef *huart)
{
  /* FE 98：立即停止，sync=0立即执行。 */
  uint8_t cmd[] = {EMM_ADDR_DEFAULT, 0xFEU, 0x98U, EMM_SYNC_DISABLE, EMM_CHECKSUM_FIXED};
  return HAL_UART_Transmit(huart, cmd, sizeof(cmd), EMM_UART_TIMEOUT_MS);
}

static uint8_t MKS_Checksum(const uint8_t *buffer, uint8_t size)
{
  uint8_t i;
  uint16_t sum = 0U;

  for (i = 0U; i < size; i++)
  {
    sum += buffer[i];
  }

  return (uint8_t)(sum & 0xFFU);
}

static HAL_StatusTypeDef MKS_SendCommand(uint8_t *cmd, uint8_t size)
{
  if (g_x_uart == NULL)
  {
    return HAL_ERROR;
  }

  cmd[size - 1U] = MKS_Checksum(cmd, (uint8_t)(size - 1U));
  return HAL_UART_Transmit(g_x_uart, cmd, size, EMM_UART_TIMEOUT_MS);
}

static HAL_StatusTypeDef MKS_SetWorkMode(uint8_t mode)
{
  /* 82H：设置Mode，02H为CR_vFOC脉冲FOC模式。 */
  uint8_t cmd[] = {MKS_FRAME_HEAD, MKS_ADDR_DEFAULT, 0x82U, mode, 0x00U};
  return MKS_SendCommand(cmd, sizeof(cmd));
}

static HAL_StatusTypeDef MKS_SetEnableLevelLow(void)
{
  /* 85H：设置En有效电平，00H为低电平有效；不要用02H Hold。 */
  uint8_t cmd[] = {MKS_FRAME_HEAD, MKS_ADDR_DEFAULT, 0x85U, MKS_EN_LEVEL_LOW, 0x00U};
  return MKS_SendCommand(cmd, sizeof(cmd));
}

static HAL_StatusTypeDef MKS_SetNearestHomeMode(void)
{
  /* 9AH：单圈回零参数，NearMode+保持已有零点，K1时只回到已设零点。 */
  uint8_t cmd[] = {
    MKS_FRAME_HEAD,
    MKS_ADDR_DEFAULT,
    0x9AU,
    MKS_HOME_NEAR_MODE,
    MKS_HOME_KEEP_ZERO,
    MKS_HOME_SPEED,
    MKS_HOME_DIR_CW,
    0x00U
  };
  return MKS_SendCommand(cmd, sizeof(cmd));
}

static HAL_StatusTypeDef MKS_GoNearestHome(void)
{
  /* 91H：执行原点回零；配合9AH的单圈就近回零参数使用。 */
  uint8_t cmd[] = {MKS_FRAME_HEAD, MKS_ADDR_DEFAULT, 0x91U, MKS_GO_HOME_ORIGIN, 0x00U};
  return MKS_SendCommand(cmd, sizeof(cmd));
}

static HAL_StatusTypeDef MKS_ConfigPulseMode(void)
{
  HAL_StatusTypeDef status;

  status = MKS_SetEnableLevelLow();
  if (status != HAL_OK)
  {
    return status;
  }

  return MKS_SetWorkMode(MKS_MODE_CR_VFOC);
}

static void MKS_SetXEnable(uint8_t enable)
{
  /* MKS En=L：PE2拉低使能，拉高释放，复位后保持可手转。 */
  HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, (enable != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void EMM_CheckK1Start(void)
{
  static uint8_t stable_level = GPIO_PIN_SET;
  static uint8_t last_sample = GPIO_PIN_SET;
  static uint16_t debounce_ms;

  uint8_t sample = (uint8_t)HAL_GPIO_ReadPin(KEY_K1_GPIO_Port, KEY_K1_Pin);

  if (sample != last_sample)
  {
    last_sample = sample;
    debounce_ms = 0U;
    return;
  }

  if (debounce_ms < EMM_KEY_DEBOUNCE_MS)
  {
    debounce_ms++;
    return;
  }

  if (sample != stable_level)
  {
    stable_level = sample;

    if ((stable_level == GPIO_PIN_RESET) && (g_seq_state == EMM_SEQ_IDLE))
    {
      /* K1按下后先使能两块驱动板，再触发就近回零。 */
      g_seq_state = EMM_SEQ_ENABLE_Y;
      g_seq_delay_ms = 0U;
    }
  }
}

HAL_StatusTypeDef EMM_Stepper_Init(UART_HandleTypeDef *y_uart, UART_HandleTypeDef *x_uart)
{
  HAL_StatusTypeDef status_y;
  HAL_StatusTypeDef status_x;

  g_y_uart = y_uart;
  g_x_uart = x_uart;
  g_seq_state = EMM_SEQ_IDLE;
  g_seq_delay_ms = 0U;

  /* 复位后先失能驱动板，电机保持可手转状态。 */
  MKS_SetXEnable(0U);
  status_y = EMM_SendEnable(g_y_uart, 0U);
  status_x = MKS_ConfigPulseMode();

  return (status_y == HAL_OK) ? status_x : status_y;
}

void EMM_Stepper_Process1ms(void)
{
  EMM_CheckK1Start();

  if (g_seq_delay_ms > 0U)
  {
    g_seq_delay_ms--;
    return;
  }

  switch (g_seq_state)
  {
    case EMM_SEQ_ENABLE_Y:
      (void)EMM_SendEnable(g_y_uart, 1U);
      g_seq_state = EMM_SEQ_ENABLE_X;
      g_seq_delay_ms = EMM_CMD_GAP_MS;
      break;

    case EMM_SEQ_ENABLE_X:
      MKS_SetXEnable(1U);
      g_seq_state = EMM_SEQ_HOME_Y;
      g_seq_delay_ms = EMM_ENABLE_SETTLE_MS;
      break;

    case EMM_SEQ_HOME_Y:
      (void)EMM_SendNearestHome(g_y_uart);
      g_seq_state = EMM_SEQ_HOME_X;
      g_seq_delay_ms = EMM_CMD_GAP_MS;
      break;

    case EMM_SEQ_HOME_X:
      (void)MKS_SetNearestHomeMode();
      (void)MKS_GoNearestHome();
      g_seq_state = EMM_SEQ_RESTORE_X_PULSE_MODE;
      g_seq_delay_ms = EMM_CMD_GAP_MS;
      break;

    case EMM_SEQ_RESTORE_X_PULSE_MODE:
      (void)MKS_SetWorkMode(MKS_MODE_CR_VFOC);
      g_seq_state = EMM_SEQ_IDLE;
      break;

    default:
      break;
  }
}

uint8_t EMM_Stepper_IsBusy(void)
{
  return (g_seq_state == EMM_SEQ_IDLE) ? 0U : 1U;
}

HAL_StatusTypeDef EMM_Stepper_EnableX(uint8_t enable)
{
  MKS_SetXEnable(enable);
  return HAL_OK;
}

HAL_StatusTypeDef EMM_Stepper_EnableY(uint8_t enable)
{
  if (g_y_uart == NULL)
  {
    return HAL_ERROR;
  }

  return EMM_SendEnable(g_y_uart, (enable != 0U) ? 1U : 0U);
}

HAL_StatusTypeDef EMM_Stepper_RunXSpeed(uint8_t dir, uint16_t speed_rpm, uint8_t acc)
{
  (void)acc;
  return PulseStepper_RunXSpeed(dir, speed_rpm);
}

HAL_StatusTypeDef EMM_Stepper_RunYSpeed(uint8_t dir, uint16_t speed_rpm, uint8_t acc)
{
  if (g_y_uart == NULL)
  {
    return HAL_ERROR;
  }

  return EMM_SendSpeedMode(g_y_uart, dir, speed_rpm, acc);
}

HAL_StatusTypeDef EMM_Stepper_StopX(void)
{
  PulseStepper_StopX();
  return HAL_OK;
}

HAL_StatusTypeDef EMM_Stepper_StopY(void)
{
  if (g_y_uart == NULL)
  {
    return HAL_ERROR;
  }

  return EMM_SendStop(g_y_uart);
}
