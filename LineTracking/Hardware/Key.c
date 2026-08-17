#include "stm32f10x.h"
#include "Key.h"

#define KEY_DEBOUNCE_COUNT     3U

typedef struct
{
	uint8_t last_sample;
	uint8_t stable_state;
	uint8_t same_count;
} KeyDebounce_t;

static KeyDebounce_t Key_SwitchState;
static KeyDebounce_t Key_ConfirmState;

static uint8_t Key_Update(KeyDebounce_t *key, uint8_t current_sample)
{
	if (current_sample == key->last_sample)
	{
		if (key->same_count < KEY_DEBOUNCE_COUNT)
		{
			key->same_count++;
		}
	}
	else
	{
		key->last_sample = current_sample;
		key->same_count = 1U;
	}

	if ((key->same_count >= KEY_DEBOUNCE_COUNT) &&
		(key->stable_state != current_sample))
	{
		key->stable_state = current_sample;

		/* 只在按下沿产生一次事件，松开时仅更新稳定状态。 */
		if (current_sample != 0U)
		{
			return 1U;
		}
	}

	return 0U;
}

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/*GPIO初始化*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*
	 * 按键另一端接 3.3V：
	 * PB12 松开为低，按下为高，对应 KEY_MODE_SWITCH；
	 * PB13 松开为低，按下为高，对应 KEY_MODE_CONFIRM。
	 */
	Key_SwitchState.last_sample = 0U;
	Key_SwitchState.stable_state = 0U;
	Key_SwitchState.same_count = 0U;

	Key_ConfirmState.last_sample = 0U;
	Key_ConfirmState.stable_state = 0U;
	Key_ConfirmState.same_count = 0U;
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：KEY_NONE、KEY_MODE_SWITCH 或 KEY_MODE_CONFIRM
  * 注意事项：应每10ms调用一次；连续稳定3次后确认按下，全程不阻塞
  */
uint8_t Key_GetNum(void)
{
	uint8_t switch_pressed;
	uint8_t confirm_pressed;

	switch_pressed = Key_Update(
		&Key_SwitchState,
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12));

	confirm_pressed = Key_Update(
		&Key_ConfirmState,
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13));

	/* 两键同时按下时，确认键优先。 */
	if (confirm_pressed != 0U)
	{
		return KEY_MODE_CONFIRM;
	}

	if (switch_pressed != 0U)
	{
		return KEY_MODE_SWITCH;
	}

	return KEY_NONE;
}
