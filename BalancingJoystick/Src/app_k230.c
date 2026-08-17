#include "app_k230.h"
#include <stdlib.h>
#include <string.h>

#define APP_K230_LINE_MAX 64U

static UART_HandleTypeDef *s_k230_huart;
static uint8_t s_k230_rx_byte;
static char s_k230_line[APP_K230_LINE_MAX];
static char s_k230_parse[APP_K230_LINE_MAX];
static volatile uint8_t s_k230_index;
static volatile uint8_t s_k230_line_ready;
static AppK230_Target_t s_k230_target;
static uint8_t s_k230_target_valid;
static uint32_t s_k230_target_tick;
static uint32_t s_k230_frames;
static uint32_t s_k230_lost_frames;

static int16_t AppK230_ParseInt(char **cursor)
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

static uint8_t AppK230_ParseLine(const char *line)
{
  char *cursor;

  if ((line[0] == 'N') || (strncmp(line, "STOP", 4U) == 0))
  {
    s_k230_target_valid = 0U;
    s_k230_target_tick = HAL_GetTick();
    s_k230_lost_frames++;
    return APP_K230_EVENT_NO_TARGET;
  }

  if (line[0] != 'T')
  {
    return APP_K230_EVENT_NONE;
  }

  strncpy(s_k230_parse, line, sizeof(s_k230_parse) - 1U);
  s_k230_parse[sizeof(s_k230_parse) - 1U] = '\0';
  cursor = &s_k230_parse[1];

  s_k230_target.cx = AppK230_ParseInt(&cursor);
  s_k230_target.cy = AppK230_ParseInt(&cursor);
  s_k230_target.dx = AppK230_ParseInt(&cursor);
  s_k230_target.dy = AppK230_ParseInt(&cursor);
  s_k230_target.w = AppK230_ParseInt(&cursor);
  s_k230_target.h = AppK230_ParseInt(&cursor);
  s_k230_target.pos_mm = AppK230_ParseInt(&cursor);
  s_k230_target_valid = 1U;
  s_k230_target_tick = HAL_GetTick();
  s_k230_frames++;

  return APP_K230_EVENT_TARGET;
}

void AppK230_Init(UART_HandleTypeDef *huart)
{
  s_k230_huart = huart;
  s_k230_index = 0U;
  s_k230_line_ready = 0U;
  s_k230_target_valid = 0U;
  s_k230_target_tick = HAL_GetTick();
}

void AppK230_StartRx(void)
{
  if (s_k230_huart == NULL)
  {
    return;
  }

  s_k230_index = 0U;
  s_k230_line_ready = 0U;
  (void)HAL_UART_Receive_IT(s_k230_huart, &s_k230_rx_byte, 1U);
}

uint8_t AppK230_ProcessLine(void)
{
  char line[APP_K230_LINE_MAX];

  if (s_k230_line_ready == 0U)
  {
    return APP_K230_EVENT_NONE;
  }

  __disable_irq();
  strncpy(line, s_k230_line, sizeof(line) - 1U);
  line[sizeof(line) - 1U] = '\0';
  s_k230_line_ready = 0U;
  __enable_irq();

  return AppK230_ParseLine(line);
}

void AppK230_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((s_k230_huart == NULL) || (huart->Instance != s_k230_huart->Instance))
  {
    return;
  }

  if ((s_k230_rx_byte == '\n') || (s_k230_rx_byte == '\r'))
  {
    if ((s_k230_index > 0U) && (s_k230_line_ready == 0U))
    {
      s_k230_line[s_k230_index] = '\0';
      s_k230_line_ready = 1U;
    }
    s_k230_index = 0U;
  }
  else if (s_k230_index < (APP_K230_LINE_MAX - 1U))
  {
    s_k230_line[s_k230_index++] = (char)s_k230_rx_byte;
  }
  else
  {
    s_k230_index = 0U;
  }

  (void)HAL_UART_Receive_IT(s_k230_huart, &s_k230_rx_byte, 1U);
}

void AppK230_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((s_k230_huart == NULL) || (huart->Instance != s_k230_huart->Instance))
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(s_k230_huart);
  AppK230_StartRx();
}

void AppK230_Invalidate(void)
{
  s_k230_target_valid = 0U;
}

uint8_t AppK230_IsTargetValid(void)
{
  return s_k230_target_valid;
}

uint8_t AppK230_HasFreshTarget(uint32_t timeout_ms)
{
  return ((s_k230_target_valid != 0U) &&
          ((HAL_GetTick() - s_k230_target_tick) <= timeout_ms)) ? 1U : 0U;
}

uint32_t AppK230_GetAgeMs(void)
{
  return HAL_GetTick() - s_k230_target_tick;
}

uint32_t AppK230_GetFrames(void)
{
  return s_k230_frames;
}

uint32_t AppK230_GetLostFrames(void)
{
  return s_k230_lost_frames;
}

int16_t AppK230_GetPosMm(void)
{
  return s_k230_target.pos_mm;
}

AppK230_Target_t AppK230_GetTarget(void)
{
  return s_k230_target;
}
