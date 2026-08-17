#ifndef __APP_MODE_H
#define __APP_MODE_H

#include <stdint.h>

/**
  * @brief  初始化菜单模式选择按键（PB12/PB13）
  * @param  无
  * @retval 无
  */
void App_Mode_Init(void);

/**
  * @brief  上电选择系统运行模式
  * @param  无
  * @retval 返回选定的模式编号（1~6）
  */
uint8_t App_Mode_Select(void);

/**
  * @brief  根据选定模式执行目标任务（死循环运行）
  * @param  mode 需要运行的模式编号
  * @retval 无
  */
void App_Mode_Run(uint8_t mode);

#endif /* __APP_MODE_H */
