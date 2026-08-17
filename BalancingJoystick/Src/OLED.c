#include "OLED.h"
#include "OLED_Font.h"
#include "i2c.h"
#include <string.h>

#define OLED_ADDR        (0x3CU << 1) /* SSD1306常用7位地址0x3C，HAL使用左移后的8位地址 */
#define OLED_WIDTH       128U
#define OLED_PAGES       8U
#define OLED_LINE_COUNT  4U
#define OLED_COL_COUNT   16U

static uint8_t g_oled_ok;

static HAL_StatusTypeDef OLED_WriteCommand(uint8_t command)
{
  uint8_t buffer[2] = {0x00U, command};
  return HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, buffer, sizeof(buffer), 20U);
}

static HAL_StatusTypeDef OLED_WriteDataBuffer(const uint8_t *data, uint16_t length)
{
  uint8_t packet[17];
  uint16_t offset = 0U;
  uint16_t chunk;

  packet[0] = 0x40U;
  while (offset < length)
  {
    chunk = (uint16_t)(((length - offset) > 16U) ? 16U : (length - offset));
    memcpy(&packet[1], &data[offset], chunk);
    if (HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDR, packet, (uint16_t)(chunk + 1U), 20U) != HAL_OK)
    {
      g_oled_ok = 0U;
      return HAL_ERROR;
    }
    offset = (uint16_t)(offset + chunk);
  }

  g_oled_ok = 1U;
  return HAL_OK;
}

static HAL_StatusTypeDef OLED_WriteData(uint8_t data)
{
  return OLED_WriteDataBuffer(&data, 1U);
}

static void OLED_SetCursor(uint8_t page, uint8_t x)
{
  (void)OLED_WriteCommand((uint8_t)(0xB0U | page));
  (void)OLED_WriteCommand((uint8_t)(0x10U | ((x & 0xF0U) >> 4U)));
  (void)OLED_WriteCommand((uint8_t)(0x00U | (x & 0x0FU)));
}

static uint32_t OLED_Pow(uint32_t x, uint32_t y)
{
  uint32_t result = 1U;

  while (y-- != 0U)
  {
    result *= x;
  }

  return result;
}

HAL_StatusTypeDef OLED_Init(void)
{
  static const uint8_t init_cmds[] = {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
    0xDBU, 0x30U, 0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU
  };
  uint8_t i;

  HAL_Delay(100U);

  if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_ADDR, 2U, 20U) != HAL_OK)
  {
    g_oled_ok = 0U;
    return HAL_ERROR;
  }

  for (i = 0U; i < sizeof(init_cmds); i++)
  {
    if (OLED_WriteCommand(init_cmds[i]) != HAL_OK)
    {
      g_oled_ok = 0U;
      return HAL_ERROR;
    }
  }

  g_oled_ok = 1U;
  OLED_Clear();
  return HAL_OK;
}

void OLED_Clear(void)
{
  uint8_t zeros[OLED_WIDTH];
  uint8_t page;

  memset(zeros, 0, sizeof(zeros));
  for (page = 0U; page < OLED_PAGES; page++)
  {
    OLED_SetCursor(page, 0U);
    (void)OLED_WriteDataBuffer(zeros, sizeof(zeros));
  }
}

void OLED_ClearLine(uint8_t line)
{
  uint8_t zeros[OLED_WIDTH];
  uint8_t top_page;

  if ((line == 0U) || (line > OLED_LINE_COUNT))
  {
    return;
  }

  memset(zeros, 0, sizeof(zeros));
  top_page = (uint8_t)((line - 1U) * 2U);
  OLED_SetCursor(top_page, 0U);
  (void)OLED_WriteDataBuffer(zeros, sizeof(zeros));
  OLED_SetCursor((uint8_t)(top_page + 1U), 0U);
  (void)OLED_WriteDataBuffer(zeros, sizeof(zeros));
}

void OLED_ShowChar(uint8_t line, uint8_t column, char ch)
{
  uint8_t i;
  uint8_t index;
  uint8_t x;
  uint8_t top_page;

  if ((line == 0U) || (line > OLED_LINE_COUNT) || (column == 0U) || (column > OLED_COL_COUNT))
  {
    return;
  }

  if ((ch < ' ') || (ch > '~'))
  {
    ch = ' ';
  }

  index = (uint8_t)(ch - ' ');
  x = (uint8_t)((column - 1U) * 8U);
  top_page = (uint8_t)((line - 1U) * 2U);

  OLED_SetCursor(top_page, x);
  for (i = 0U; i < 8U; i++)
  {
    (void)OLED_WriteData(OLED_F8x16[index][i]);
  }

  OLED_SetCursor((uint8_t)(top_page + 1U), x);
  for (i = 0U; i < 8U; i++)
  {
    (void)OLED_WriteData(OLED_F8x16[index][i + 8U]);
  }
}

void OLED_ShowString(uint8_t line, uint8_t column, const char *string)
{
  uint8_t i = 0U;

  if (string == NULL)
  {
    return;
  }

  while ((string[i] != '\0') && ((column + i) <= OLED_COL_COUNT))
  {
    OLED_ShowChar(line, (uint8_t)(column + i), string[i]);
    i++;
  }
}

void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
  uint8_t i;

  for (i = 0U; i < length; i++)
  {
    OLED_ShowChar(line, (uint8_t)(column + i),
                  (char)((number / OLED_Pow(10U, (uint32_t)(length - i - 1U))) % 10U + '0'));
  }
}

void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length)
{
  uint8_t i;
  uint32_t abs_number;

  if (number >= 0)
  {
    OLED_ShowChar(line, column, '+');
    abs_number = (uint32_t)number;
  }
  else
  {
    OLED_ShowChar(line, column, '-');
    abs_number = (uint32_t)(-number);
  }

  for (i = 0U; i < length; i++)
  {
    OLED_ShowChar(line, (uint8_t)(column + i + 1U),
                  (char)((abs_number / OLED_Pow(10U, (uint32_t)(length - i - 1U))) % 10U + '0'));
  }
}

void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
  uint8_t i;
  uint8_t single;

  for (i = 0U; i < length; i++)
  {
    single = (uint8_t)((number / OLED_Pow(16U, (uint32_t)(length - i - 1U))) % 16U);
    OLED_ShowChar(line, (uint8_t)(column + i), (char)(single < 10U ? (single + '0') : (single - 10U + 'A')));
  }
}

void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
  uint8_t i;

  for (i = 0U; i < length; i++)
  {
    OLED_ShowChar(line, (uint8_t)(column + i),
                  (char)((number / OLED_Pow(2U, (uint32_t)(length - i - 1U))) % 2U + '0'));
  }
}

uint8_t OLED_IsOk(void)
{
  return g_oled_ok;
}
