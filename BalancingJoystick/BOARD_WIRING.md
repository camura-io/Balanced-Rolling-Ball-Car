# H750_single_stepper_board 接线说明

## 固件工程

- MCU: STM32H750VBT6
- 工程: `D:\CodexAI\diansaiH750\H750_single_stepper_board`
- IOC: `D:\CodexAI\diansaiH750\H750_single_stepper_board\H750_single_stepper_board.ioc`
- Keil: `D:\CodexAI\diansaiH750\H750_single_stepper_board\MDK-ARM\H750_single_stepper_board.uvprojx`

## 已确定接口

| 功能 | STM32 引脚 | 外设 | 连接 |
| --- | --- | --- | --- |
| OLED SCL | PB10 | I2C2_SCL | OLED SCL |
| OLED SDA | PB11 | I2C2_SDA | OLED SDA |
| K230 TX | PA2 | USART2_TX | 接 K230 RX |
| K230 RX | PA3 | USART2_RX | 接 K230 TX |
| K230 电源 | 3V3/GND | - | K230 串口侧共地 |

## 新增接口分配

| 功能 | STM32 引脚 | 外设 | 连接 |
| --- | --- | --- | --- |
| 张大头电机 STEP/STP | PB4 | TIM3_CH1 PWM | 驱动器 STP/STEP |
| 张大头电机 DIR | PE0 | GPIO Output | 驱动器 DIR |
| 张大头电机 EN | PB8 | GPIO Output | 驱动器 EN |
| 张大头电机串口 TX | PC10 | USART3_TX | 驱动器 RX |
| 张大头电机串口 RX | PC11 | USART3_RX | 驱动器 TX |
| MPU6050 SCL | PB6 | I2C1_SCL | MPU6050 SCL |
| MPU6050 SDA | PB7 | I2C1_SDA | MPU6050 SDA |
| IMU901 TX | PA9 | USART1_TX, 115200 | IMU901 RX |
| IMU901 RX | PA10 | USART1_RX, 115200 | IMU901 TX |
| ICM42688 SCLK | PA5 | SPI1_SCK | ICM42688 SCL/SCLK |
| ICM42688 MISO | PA6 | SPI1_MISO | ICM42688 AD0/MISO |
| ICM42688 MOSI | PA7 | SPI1_MOSI | ICM42688 SDA/MOSI |
| ICM42688 CS | PE4 | GPIO Output | ICM42688 CS |
| ICM42688 INT1 | PE5 | EXTI9_5 | ICM42688 INT1 |
| K1 | PE7 | GPIO Input Pull-up | 按键到 GND |
| K2 | PE9 | GPIO Input Pull-up | 按键到 GND |
| K3 | PE11 | GPIO Input Pull-up | 按键到 GND |
| K4 | PE13 | GPIO Input Pull-up | 按键到 GND |
| K5 | PE3 | GPIO Input Pull-up | 旧调试K1改名，按键到 GND |
| K6 | PC5 | GPIO Input Pull-up | 旧调试K2改名，按键到 GND |
| RUN_LED | PA1 | GPIO Output | 状态 LED |
| SWDIO | PA13 | SYS Debug | 调试口 |
| SWCLK | PA14 | SYS Debug | 调试口 |

## 张大头步进电机脉冲接线

当前工程按张大头步进电机的脉冲控制方式走，`USART3` 只作为电机串口预留，不再发送其他型号驱动器的专用配置帧。
张大头 Emm_V5.0 说明书第 27 页 STM32 脉冲控制接线图确认: `COM -> 3.3V`。

| 张大头电机端子 | 连接 |
| --- | --- |
| COM | STM32 板 3V3 |
| STP/STEP | PB4 |
| DIR | PE0 |
| EN | PB8 |
| GND | STM32 GND |
| TTL RX | PC10 |
| TTL TX | PC11 |

EN 当前按低电平使能写: `PB8=0` 使能锁电机，`PB8=1` 失能可手转。复位默认失能。如果你的张大头驱动 EN 有效电平相反，只需要改固件宏/逻辑，不影响 PCB 引脚分配。

## I2C 注意

- I2C1 和 I2C2 都是开漏输出。
- PCB 上建议每条 I2C 总线各放一组 4.7k 上拉到 3V3。
- 如果 OLED/MPU6050 模块本身已经带上拉，板上上拉可预留焊盘或 0R/NC 选择。

## 当前程序行为

- 复位后: 电机失能，LED 按 1s 节拍闪烁。
- K1: PE7，电机使能回零，随后采 MS901M 零点并启动中心稳定。
- K2: PE9，启动赛题第三问 `0 -> +5cm -> -5cm`。
- K3: PE11，电机使能回零后启动纯视觉中心稳定。
- K4: PE13，第一次按下只回零静止，第二次按下失能释放电机。
- K5: PE3。K1 或 K3 启动稳定后，每按一次将稳定目标加 `+1cm`，可调到 `+12cm`。
- K6: PC5。K1 或 K3 启动稳定后，每按一次将稳定目标减 `-1cm`，可调到 `-12cm`。
- OLED: 稳定目标以 `T:+010mm` 格式显示；K2 第三问仍固定使用 `+5cm -> -5cm`，K5/K6 不改动其独立目标。
- K230 协议: `T,cx,cy,dx,dy,w,h\n`、`N\n`、`STOP\n`。
- OLED 第一行 5 个状态块: MPU6050、IMU901、K230目标、电机使能、ICM42688。
