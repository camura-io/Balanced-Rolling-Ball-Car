#include "icm42688.h"
#include "spi.h"
#include "stm32h7xx_hal_spi_ex.h"
#include <string.h>

#define ICM42688_DEVICE_CONFIG      0x11U
#define ICM42688_INT_CONFIG         0x14U
#define ICM42688_TEMP_DATA1         0x1DU
#define ICM42688_ACCEL_DATA_X1      0x1FU
#define ICM42688_PWR_MGMT0          0x4EU
#define ICM42688_GYRO_CONFIG0       0x4FU
#define ICM42688_ACCEL_CONFIG0      0x50U
#define ICM42688_INT_SOURCE0        0x65U
#define ICM42688_WHO_AM_I           0x75U
#define ICM42688_REG_BANK_SEL       0x76U

#define ICM42688_ID                 0x47U
#define ICM42688_SPI_READ           0x80U
#define ICM42688_SPI_DEFAULT_MODE   3U /* 资料例程是Mode3；若硬件实际不同，上电只用WHO_AM_I探测后锁定。 */
#define ICM42688_POLL_PERIOD_MS     10U
#define ICM42688_RETRY_PERIOD_MS    1000U
#define ICM42688_SPI_TIMEOUT_MS     50U
#define ICM42688_READ_STYLE_BURST   1U
#define ICM42688_INVALID_RAW        ((int16_t)-32768)

static ICM42688_Data_t g_icm42688;
static volatile uint8_t g_icm42688_data_ready;

static int16_t ICM42688_ReadInt16BE(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void ICM42688_Select(uint8_t select)
{
  HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin,
                    (select != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void ICM42688_BusDelay(void)
{
  volatile uint32_t i;

  for (i = 0U; i < 80U; i++)
  {
    __NOP();
  }
}

static HAL_StatusTypeDef ICM42688_SetSpiMode(uint8_t mode)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t tx = 0xFFU;
  uint8_t rx = 0U;

  ICM42688_Select(0U);
  (void)HAL_SPI_DeInit(&hspi1);

  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* 上电探测先降速，约504kHz */
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_04CYCLE;

  switch (mode & 0x03U)
  {
    case 0U:
      hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
      hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
      break;
    case 1U:
      hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
      hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
      break;
    case 2U:
      hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
      hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
      break;
    default:
      hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
      hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
      break;
  }

  status = HAL_SPI_Init(&hspi1);
  g_icm42688.spi_status = (uint8_t)status;
  if (status == HAL_OK)
  {
    g_icm42688.active_mode = (uint8_t)(mode & 0x03U);
    (void)HAL_SPIEx_FlushRxFifo(&hspi1);
    (void)HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1U, ICM42688_SPI_TIMEOUT_MS);
  }
  return status;
}

static HAL_StatusTypeDef ICM42688_WriteReg(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status;

  (void)HAL_SPIEx_FlushRxFifo(&hspi1);
  ICM42688_Select(1U);
  ICM42688_BusDelay();
  status = HAL_SPI_Transmit(&hspi1, &reg, 1U, ICM42688_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Transmit(&hspi1, &value, 1U, ICM42688_SPI_TIMEOUT_MS);
  }
  ICM42688_BusDelay();
  ICM42688_Select(0U);
  g_icm42688.spi_status = (uint8_t)status;
  return status;
}

static HAL_StatusTypeDef ICM42688_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t tx[16];
  uint8_t rx[16];
  uint8_t i;

  if ((data == NULL) || (len == 0U) || (len > 15U))
  {
    return HAL_ERROR;
  }

  memset(tx, 0, sizeof(tx));
  memset(rx, 0, sizeof(rx));
  tx[0] = (uint8_t)(reg | ICM42688_SPI_READ);
  for (i = 1U; i <= len; i++)
  {
    tx[i] = (len == 1U) ? 0xFFU : 0x00U;
  }

  (void)HAL_SPIEx_FlushRxFifo(&hspi1);
  ICM42688_Select(1U);
  ICM42688_BusDelay();
  status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(len + 1U), ICM42688_SPI_TIMEOUT_MS);
  ICM42688_BusDelay();
  ICM42688_Select(0U);
  g_icm42688.spi_status = (uint8_t)status;
  if (status == HAL_OK)
  {
    memcpy(data, &rx[1], len);
  }
  return status;
}

static HAL_StatusTypeDef ICM42688_ReadReg(uint8_t reg, uint8_t *value)
{
  return ICM42688_ReadRegs(reg, value, 1U);
}

static HAL_StatusTypeDef ICM42688_ProbeSpiMode(uint8_t *who)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t mode;
  uint8_t id = 0U;

  if (who == NULL)
  {
    return HAL_ERROR;
  }

  *who = 0U;
  for (mode = 0U; mode < 4U; mode++)
  {
    g_icm42688.probe_id[mode] = 0U;
    g_icm42688.shifted_id[mode] = 0U;
    if (ICM42688_SetSpiMode(mode) == HAL_OK)
    {
      id = 0U;
      status = ICM42688_ReadReg(ICM42688_WHO_AM_I, &id);
      g_icm42688.probe_id[mode] = id;
      if ((status == HAL_OK) && (id == ICM42688_ID))
      {
        g_icm42688.active_mode = mode;
        g_icm42688.read_style = ICM42688_READ_STYLE_BURST;
        *who = id;
        return HAL_OK;
      }
    }
  }

  (void)ICM42688_SetSpiMode(ICM42688_SPI_DEFAULT_MODE);
  g_icm42688.active_mode = ICM42688_SPI_DEFAULT_MODE;
  g_icm42688.read_style = ICM42688_READ_STYLE_BURST;
  *who = g_icm42688.probe_id[ICM42688_SPI_DEFAULT_MODE];
  return HAL_ERROR;
}

static HAL_StatusTypeDef ICM42688_ReadRaw(void)
{
  uint8_t raw[12];
  uint8_t temp_raw[2];
  uint8_t who = 0U;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;

  if (g_icm42688.who_am_i != ICM42688_ID)
  {
    if (ICM42688_ReadReg(ICM42688_WHO_AM_I, &who) != HAL_OK)
    {
      g_icm42688.ok = 0U;
      g_icm42688.errors++;
      return HAL_ERROR;
    }
    g_icm42688.who_am_i = who;
    if (who != ICM42688_ID)
    {
      g_icm42688.ok = 0U;
      g_icm42688.errors++;
      return HAL_ERROR;
    }
  }

  /* 按资料例程从ACCEL_DATA_X1连续读12字节，避免温度字段偏移造成误判。 */
  if (ICM42688_ReadRegs(ICM42688_ACCEL_DATA_X1, raw, sizeof(raw)) != HAL_OK)
  {
    g_icm42688.ok = 0U;
    g_icm42688.errors++;
    return HAL_ERROR;
  }

  ax = ICM42688_ReadInt16BE(&raw[0]);
  ay = ICM42688_ReadInt16BE(&raw[2]);
  az = ICM42688_ReadInt16BE(&raw[4]);
  gx = ICM42688_ReadInt16BE(&raw[6]);
  gy = ICM42688_ReadInt16BE(&raw[8]);
  gz = ICM42688_ReadInt16BE(&raw[10]);
  if ((ax == ICM42688_INVALID_RAW) || (ay == ICM42688_INVALID_RAW) ||
      (az == ICM42688_INVALID_RAW) || (gx == ICM42688_INVALID_RAW) ||
      (gy == ICM42688_INVALID_RAW) || (gz == ICM42688_INVALID_RAW))
  {
    /* -32768是ICM数据无效标志，不代表SPI掉线；丢弃本帧，保留上一帧有效数据。 */
    g_icm42688.ok = 1U;
    g_icm42688.errors++;
    g_icm42688.last_tick = HAL_GetTick();
    return HAL_ERROR;
  }

  if (ICM42688_ReadRegs(ICM42688_TEMP_DATA1, temp_raw, sizeof(temp_raw)) == HAL_OK)
  {
    g_icm42688.temp = ICM42688_ReadInt16BE(temp_raw);
  }
  g_icm42688.ax = ax;
  g_icm42688.ay = ay;
  g_icm42688.az = az;
  g_icm42688.gx = gx;
  g_icm42688.gy = gy;
  g_icm42688.gz = gz;
  g_icm42688.ok = 1U;
  g_icm42688.frames++;
  g_icm42688.last_tick = HAL_GetTick();
  return HAL_OK;
}

HAL_StatusTypeDef ICM42688_Init(void)
{
  uint8_t who = 0U;
  HAL_StatusTypeDef status;

  memset(&g_icm42688, 0, sizeof(g_icm42688));
  g_icm42688.active_mode = ICM42688_SPI_DEFAULT_MODE;
  g_icm42688.read_style = ICM42688_READ_STYLE_BURST;
  g_icm42688_data_ready = 0U;
  ICM42688_Select(0U);
  HAL_Delay(10U);

  status = ICM42688_ProbeSpiMode(&who);
  g_icm42688.who_am_i = who;
  if ((status != HAL_OK) || (who != ICM42688_ID))
  {
    g_icm42688.ok = 0U;
    g_icm42688.errors++;
    g_icm42688.last_tick = HAL_GetTick();
    return HAL_ERROR;
  }

  (void)ICM42688_WriteReg(ICM42688_REG_BANK_SEL, 0x00U);
  (void)ICM42688_WriteReg(ICM42688_DEVICE_CONFIG, 0x01U);
  HAL_Delay(100U);

  status = ICM42688_ReadReg(ICM42688_WHO_AM_I, &who);
  g_icm42688.who_am_i = who;
  if ((status != HAL_OK) || (who != ICM42688_ID))
  {
    g_icm42688.ok = 0U;
    g_icm42688.errors++;
    g_icm42688.last_tick = HAL_GetTick();
    return HAL_ERROR;
  }

  /* INT1输出数据就绪脉冲；4g、1000dps、100Hz，先保守稳定。 */
  (void)ICM42688_WriteReg(ICM42688_REG_BANK_SEL, 0x00U);
  (void)ICM42688_WriteReg(ICM42688_INT_CONFIG, 0x30U);
  (void)ICM42688_WriteReg(ICM42688_INT_SOURCE0, 0x08U);
  (void)ICM42688_WriteReg(ICM42688_ACCEL_CONFIG0, (uint8_t)((0x02U << 5) | 0x08U));
  (void)ICM42688_WriteReg(ICM42688_GYRO_CONFIG0, (uint8_t)((0x01U << 5) | 0x08U));
  (void)ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x0FU);
  HAL_Delay(2U);

  return ICM42688_ReadRaw();
}

void ICM42688_Process(void)
{
  uint32_t now = HAL_GetTick();

  if ((g_icm42688.ok == 0U) || (g_icm42688.who_am_i != ICM42688_ID))
  {
    g_icm42688_data_ready = 0U;
    if ((now - g_icm42688.last_tick) >= ICM42688_RETRY_PERIOD_MS)
    {
      (void)ICM42688_Init();
    }
    return;
  }

  if ((g_icm42688_data_ready != 0U) ||
      ((now - g_icm42688.last_tick) >= ICM42688_POLL_PERIOD_MS))
  {
    g_icm42688_data_ready = 0U;
    (void)ICM42688_ReadRaw();
  }
}

void ICM42688_GetData(ICM42688_Data_t *data)
{
  if (data != NULL)
  {
    *data = g_icm42688;
  }
}

uint8_t ICM42688_IsOk(void)
{
  return ((g_icm42688.ok != 0U) && (g_icm42688.who_am_i == ICM42688_ID)) ? 1U : 0U;
}

void ICM42688_EXTI_Callback(uint16_t gpio_pin)
{
  if (gpio_pin == ICM42688_INT1_Pin)
  {
    g_icm42688_data_ready = 1U;
    g_icm42688.int_count++;
  }
}
