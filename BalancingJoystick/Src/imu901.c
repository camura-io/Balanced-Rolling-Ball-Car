#include "imu901.h"
#include <string.h>

#define IMU901_RX_BUFFER_SIZE 128U
#define IMU901_RX_BUFFER_MASK (IMU901_RX_BUFFER_SIZE - 1U)
#define IMU901_MAX_PAYLOAD    32U

#define IMU901_FRAME_HEADER   0x55U
#define IMU901_FRAME_ANGLE    0x01U
#define IMU901_FRAME_ACCEL_GYRO 0x03U
#define IMU901_FRAME_ACCEL_FSR 0x04U

#define IMU901_ACCEL_RANGE_2G 0x00U
#define IMU901_ACCEL_RANGE_4G 0x01U
#define IMU901_ACCEL_RANGE_8G 0x02U
#define IMU901_ACCEL_RANGE_16G 0x03U
#define IMU901_ACCEL_RANGE_UNKNOWN 0xFFU
#define IMU901_ACCEL_FS_DEFAULT_MG 4000L /* 读回量程前按模块出厂默认±4g解析。 */

typedef enum
{
  IMU901_WAIT_HEADER_1 = 0,
  IMU901_WAIT_HEADER_2,
  IMU901_WAIT_ID,
  IMU901_WAIT_LENGTH,
  IMU901_WAIT_PAYLOAD,
  IMU901_WAIT_CHECKSUM
} IMU901_ParseState_t;

typedef struct
{
  IMU901_ParseState_t state;
  uint8_t id;
  uint8_t length;
  uint8_t index;
  uint8_t checksum;
  uint8_t payload[IMU901_MAX_PAYLOAD];
} IMU901_Parser_t;

static UART_HandleTypeDef *g_imu_uart;
static uint8_t g_uart_rx_byte;
static volatile uint16_t g_rx_write;
static volatile uint16_t g_rx_read;
static uint8_t g_rx_buffer[IMU901_RX_BUFFER_SIZE];
static IMU901_Parser_t g_parser;
static IMU901_Data_t g_data;
static uint8_t g_accel_range_code;
static int32_t g_accel_full_scale_mg;

static void IMU901_ResetParser(uint8_t current_byte)
{
  memset(&g_parser, 0, sizeof(g_parser));

  /* 当前字节仍可能是下一帧的第一个帧头。 */
  if (current_byte == IMU901_FRAME_HEADER)
  {
    g_parser.state = IMU901_WAIT_HEADER_2;
    g_parser.checksum = IMU901_FRAME_HEADER;
  }
  else
  {
    g_parser.state = IMU901_WAIT_HEADER_1;
  }
}

static int16_t IMU901_ReadInt16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static void IMU901_UpdateAccelRange(uint8_t range_code)
{
  switch (range_code)
  {
    case IMU901_ACCEL_RANGE_2G:
      g_accel_full_scale_mg = 2000L;
      break;

    case IMU901_ACCEL_RANGE_4G:
      g_accel_full_scale_mg = 4000L;
      break;

    case IMU901_ACCEL_RANGE_8G:
      g_accel_full_scale_mg = 8000L;
      break;

    case IMU901_ACCEL_RANGE_16G:
      g_accel_full_scale_mg = 16000L;
      break;

    default:
      return;
  }

  g_accel_range_code = range_code;
}

static void IMU901_DecodeFrame(void)
{
  int32_t acc_mg;

  g_data.valid_frames++;

  if ((g_parser.id == IMU901_FRAME_ANGLE) && (g_parser.length == 6U))
  {
    /* 手册公式：有符号16位原始值乘以 180 / 32768。 */
    g_data.roll = (float)IMU901_ReadInt16(&g_parser.payload[0]) * (180.0f / 32768.0f);
    g_data.pitch = (float)IMU901_ReadInt16(&g_parser.payload[2]) * (180.0f / 32768.0f);
    g_data.yaw = (float)IMU901_ReadInt16(&g_parser.payload[4]) * (180.0f / 32768.0f);
    g_data.angle_frames++;
    g_data.last_angle_tick = HAL_GetTick();
  }
  else if ((g_parser.id == IMU901_FRAME_ACCEL_GYRO) && (g_parser.length == 12U))
  {
    /* 加速度XYZ在前6字节，小端有符号；当前只存加速度，角速度暂不用。 */
    acc_mg = ((int32_t)IMU901_ReadInt16(&g_parser.payload[0]) * g_accel_full_scale_mg) / 32768L;
    g_data.acc_x_mg = (int16_t)acc_mg;
    acc_mg = ((int32_t)IMU901_ReadInt16(&g_parser.payload[2]) * g_accel_full_scale_mg) / 32768L;
    g_data.acc_y_mg = (int16_t)acc_mg;
    acc_mg = ((int32_t)IMU901_ReadInt16(&g_parser.payload[4]) * g_accel_full_scale_mg) / 32768L;
    g_data.acc_z_mg = (int16_t)acc_mg;
    g_data.accel_frames++;
    g_data.last_accel_tick = HAL_GetTick();
  }
  else if ((g_parser.id == IMU901_FRAME_ACCEL_FSR) && (g_parser.length == 1U))
  {
    IMU901_UpdateAccelRange(g_parser.payload[0]);
  }
}

static void IMU901_ParseByte(uint8_t byte)
{
  switch (g_parser.state)
  {
    case IMU901_WAIT_HEADER_1:
      if (byte == IMU901_FRAME_HEADER)
      {
        g_parser.checksum = byte;
        g_parser.state = IMU901_WAIT_HEADER_2;
      }
      break;

    case IMU901_WAIT_HEADER_2:
      if (byte == IMU901_FRAME_HEADER)
      {
        g_parser.checksum = (uint8_t)(g_parser.checksum + byte);
        g_parser.state = IMU901_WAIT_ID;
      }
      else
      {
        IMU901_ResetParser(byte);
      }
      break;

    case IMU901_WAIT_ID:
      g_parser.id = byte;
      g_parser.checksum = (uint8_t)(g_parser.checksum + byte);
      g_parser.state = IMU901_WAIT_LENGTH;
      break;

    case IMU901_WAIT_LENGTH:
      g_parser.length = byte;
      g_parser.index = 0U;
      g_parser.checksum = (uint8_t)(g_parser.checksum + byte);

      if (g_parser.length > IMU901_MAX_PAYLOAD)
      {
        g_data.format_errors++;
        IMU901_ResetParser(byte);
      }
      else if (g_parser.length == 0U)
      {
        g_parser.state = IMU901_WAIT_CHECKSUM;
      }
      else
      {
        g_parser.state = IMU901_WAIT_PAYLOAD;
      }
      break;

    case IMU901_WAIT_PAYLOAD:
      g_parser.payload[g_parser.index++] = byte;
      g_parser.checksum = (uint8_t)(g_parser.checksum + byte);
      if (g_parser.index >= g_parser.length)
      {
        g_parser.state = IMU901_WAIT_CHECKSUM;
      }
      break;

    case IMU901_WAIT_CHECKSUM:
      if (byte == g_parser.checksum)
      {
        IMU901_DecodeFrame();
        IMU901_ResetParser(0U);
      }
      else
      {
        g_data.checksum_errors++;
        IMU901_ResetParser(byte);
      }
      break;

    default:
      IMU901_ResetParser(byte);
      break;
  }
}

HAL_StatusTypeDef IMU901_Init(UART_HandleTypeDef *huart)
{
  g_imu_uart = huart;
  g_rx_write = 0U;
  g_rx_read = 0U;
  memset(&g_data, 0, sizeof(g_data));
  g_accel_range_code = IMU901_ACCEL_RANGE_UNKNOWN;
  g_accel_full_scale_mg = IMU901_ACCEL_FS_DEFAULT_MG;
  IMU901_ResetParser(0U);

  return HAL_UART_Receive_IT(g_imu_uart, &g_uart_rx_byte, 1U);
}

HAL_StatusTypeDef IMU901_EnsureAccelRange2G(void)
{
  /* 读ACCFSR；不是2G才写2G并SAVE，避免每次复位反复擦写模块Flash。 */
  static const uint8_t read_accel_fsr[] = {0x55U, 0xAFU, 0x84U, 0x01U, 0x00U, 0x89U};
  static const uint8_t set_accel_fsr_2g[] = {0x55U, 0xAFU, 0x04U, 0x01U, 0x00U, 0x09U};
  static const uint8_t save_config[] = {0x55U, 0xAFU, 0x00U, 0x01U, 0x00U, 0x05U};
  HAL_StatusTypeDef status;
  uint32_t start_tick;

  if (g_imu_uart == NULL)
  {
    return HAL_ERROR;
  }

  g_accel_range_code = IMU901_ACCEL_RANGE_UNKNOWN;
  status = HAL_UART_Transmit(g_imu_uart, (uint8_t *)read_accel_fsr, sizeof(read_accel_fsr), 100U);
  if (status != HAL_OK)
  {
    return status;
  }

  start_tick = HAL_GetTick();
  while ((HAL_GetTick() - start_tick) < 50U)
  {
    IMU901_Process();
    if (g_accel_range_code != IMU901_ACCEL_RANGE_UNKNOWN)
    {
      break;
    }
    HAL_Delay(1U);
  }

  if (g_accel_range_code == IMU901_ACCEL_RANGE_UNKNOWN)
  {
    return HAL_TIMEOUT;
  }

  if (g_accel_range_code == IMU901_ACCEL_RANGE_2G)
  {
    return HAL_OK;
  }

  status = HAL_UART_Transmit(g_imu_uart, (uint8_t *)set_accel_fsr_2g, sizeof(set_accel_fsr_2g), 100U);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(10U);
  status = HAL_UART_Transmit(g_imu_uart, (uint8_t *)save_config, sizeof(save_config), 100U);
  if (status == HAL_OK)
  {
    /* 手册规定量程保存后需重新上电；本次仍保留读回的旧量程解析。 */
    HAL_Delay(10U);
  }
  return status;
}

HAL_StatusTypeDef IMU901_SetAngleOutput100Hz(void)
{
  /* RETURNSET=姿态角+加速度/角速度，RETURNRATE=100Hz；不写SAVE，不改模块Flash。 */
  static const uint8_t angle_accel[] = {0x55U, 0xAFU, 0x08U, 0x01U, 0x05U, 0x12U};
  static const uint8_t rate_100hz[] = {0x55U, 0xAFU, 0x0AU, 0x01U, 0x03U, 0x12U};
  HAL_StatusTypeDef status;

  if (g_imu_uart == NULL)
  {
    return HAL_ERROR;
  }

  status = HAL_UART_Transmit(g_imu_uart, (uint8_t *)angle_accel, sizeof(angle_accel), 100U);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(10U);
  return HAL_UART_Transmit(g_imu_uart, (uint8_t *)rate_100hz, sizeof(rate_100hz), 100U);
}

void IMU901_Process(void)
{
  while (g_rx_read != g_rx_write)
  {
    uint8_t byte = g_rx_buffer[g_rx_read];
    g_rx_read = (uint16_t)((g_rx_read + 1U) & IMU901_RX_BUFFER_MASK);
    IMU901_ParseByte(byte);
  }
}

void IMU901_GetData(IMU901_Data_t *data)
{
  if (data != NULL)
  {
    *data = g_data;
  }
}

uint8_t IMU901_HasFreshAngle(uint32_t timeout_ms)
{
  if (g_data.angle_frames == 0U)
  {
    return 0U;
  }

  return ((HAL_GetTick() - g_data.last_angle_tick) <= timeout_ms) ? 1U : 0U;
}

uint8_t IMU901_HasFreshAccel(uint32_t timeout_ms)
{
  if (g_data.accel_frames == 0U)
  {
    return 0U;
  }

  return ((HAL_GetTick() - g_data.last_accel_tick) <= timeout_ms) ? 1U : 0U;
}

void IMU901_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((g_imu_uart != NULL) && (huart->Instance == g_imu_uart->Instance))
  {
    uint16_t next = (uint16_t)((g_rx_write + 1U) & IMU901_RX_BUFFER_MASK);

    if (next == g_rx_read)
    {
      g_data.dropped_bytes++;
    }
    else
    {
      g_rx_buffer[g_rx_write] = g_uart_rx_byte;
      g_rx_write = next;
    }

    (void)HAL_UART_Receive_IT(g_imu_uart, &g_uart_rx_byte, 1U);
  }
}

void IMU901_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((g_imu_uart != NULL) && (huart->Instance == g_imu_uart->Instance))
  {
    __HAL_UART_CLEAR_OREFLAG(g_imu_uart);
    (void)HAL_UART_Receive_IT(g_imu_uart, &g_uart_rx_byte, 1U);
  }
}
