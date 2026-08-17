# STM32H750 固定烧录流程

## 适用工程

- 工程根目录：`D:\CodexAI\diansaiH750\H750_single_stepper_boardduan`
- MCU：STM32H750VBT6
- 固件：`MDK-ARM\H750_single_stepper_board\H750_single_stepper_board.hex`
- 烧录器：ST-LINK，序列号 `B55B5A1A000000003626EF01`
- SWD 频率：`4 MHz`

## 每次烧录前的强制规则

1. 确认本次需要烧录的就是上述 `hex`，不要误烧旧工作区或其他工程。
2. 只运行工程根目录中的 `flash_h750_4mhz_hardware_reset.ps1`。
3. 脚本必须出现 `Download verified successfully` 才算烧录成功。
4. 之后脚本必须出现 `Hard reset is performed`，让新固件开始运行。
5. 烧录失败只记录完整输出并停止，等待用户检查 ST-LINK、SWD、RST 和供电；不要自行反复重试或改变参数。

## 固定命令

脚本内部使用的、已经验证成功的命令如下：

```powershell
& 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' `
  -c port=SWD freq=4000 mode=NORMAL reset=HWrst `
  -w 'D:\CodexAI\diansaiH750\H750_single_stepper_boardduan\MDK-ARM\H750_single_stepper_board\H750_single_stepper_board.hex' -v

& 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' `
  -c port=SWD freq=4000 mode=NORMAL -hardRst
```

`STM32_Programmer_CLI` 会把 `reset=HWrst` 显示为 `Connect mode: Under Reset`，这是该工具对硬件复位参数的内部映射。不要为了消除这条提示而替换为其他连接模式；上面的命令已连续校验成功。

## 禁止项

- 不使用 `-e all`、全片擦除或任何手动擦除命令。
- 不修改 Option Bytes、RDP、调试锁、写保护或读保护。
- 不使用 J-Link。
- 不手动指定 `mode=UR` 或 `mode=HWRSTPULSE`。
- 不降低 SWD 频率，不改为软件复位，不换固件路径。
- 不在未获得用户明确同意时烧录。

## 执行方式

在工程根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\flash_h750_4mhz_hardware_reset.ps1
```

