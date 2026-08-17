#include "stm32f10x.h"
#include "pwm.h"
#include "encoder.h"
#include "OLED.h"
#include "Timer1.h"
#include "IR_I2C.h"
#include "App_Mode.h"

int main(void)
{
    uint8_t selected_mode;

    /* 0. NVIC 优先级分组：2 位抢占 + 2 位子优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 1. 底层外设初始化 */
    PWM_Int(7199, 0);          /* 电机 PWM 初始配置 */
    Encoder_Init_Tim2();       /* 左轮编码器 A */
    Encoder_Init_Tim4();       /* 右轮编码器 B */
    OLED_Init();               /* PB8=SCL，PB9=SDA */
    IR_I2C_Init();             /* PB10=SCL，PB11=SDA */
    App_Mode_Init();           /* PB12=模式切换，PB13=确认 */

    /* 72MHz 主频下产生 10ms 系统控制中断 */
    Timer1_Init(100 - 1, 7200 - 1);

    /* 2. 上电菜单轮询与模式选择 */
    selected_mode = App_Mode_Select();

    /* 3. 运行对应模式的任务流程（内置死循环） */
    App_Mode_Run(selected_mode);

    while (1)
    {
    }
}
