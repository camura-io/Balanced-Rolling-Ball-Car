#include "IR_I2C.h"

#define IR_I2C_PORT               GPIOB
#define IR_I2C_SCL_PIN            GPIO_Pin_10
#define IR_I2C_SDA_PIN            GPIO_Pin_11
#define IR_I2C_PERIPHERAL         I2C2

/* 72MHz 主频下的有限轮询超时，任何失败都不能无限阻塞。 */
#define IR_I2C_TIMEOUT_COUNT      20000U

static uint8_t IR_I2C_WaitEvent(uint32_t event)
{
    uint32_t timeout = IR_I2C_TIMEOUT_COUNT;

    while (I2C_CheckEvent(IR_I2C_PERIPHERAL, event) == ERROR)
    {
        if (timeout-- == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t IR_I2C_WaitBusIdle(void)
{
    uint32_t timeout = IR_I2C_TIMEOUT_COUNT;

    while (I2C_GetFlagStatus(IR_I2C_PERIPHERAL, I2C_FLAG_BUSY) != RESET)
    {
        if (timeout-- == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t IR_I2C_WaitRxNotEmpty(void)
{
    uint32_t timeout = IR_I2C_TIMEOUT_COUNT;

    while (I2C_GetFlagStatus(IR_I2C_PERIPHERAL, I2C_FLAG_RXNE) == RESET)
    {
        if (timeout-- == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

static void IR_I2C_Abort(void)
{
    I2C_GenerateSTOP(IR_I2C_PERIPHERAL, ENABLE);
    I2C_AcknowledgeConfig(IR_I2C_PERIPHERAL, ENABLE);
}

static uint8_t IR_I2C_WriteRegister(uint8_t reg, uint8_t value)
{
    if (!IR_I2C_WaitBusIdle())
    {
        return 0U;
    }

    I2C_GenerateSTART(IR_I2C_PERIPHERAL, ENABLE);
    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
    {
        IR_I2C_Abort();
        return 0U;
    }

    I2C_Send7bitAddress(IR_I2C_PERIPHERAL,
                        (uint8_t)(IR_I2C_ADDRESS_7BIT << 1),
                        I2C_Direction_Transmitter);
    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        IR_I2C_Abort();
        return 0U;
    }

    I2C_SendData(IR_I2C_PERIPHERAL, reg);
    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        IR_I2C_Abort();
        return 0U;
    }

    I2C_SendData(IR_I2C_PERIPHERAL, value);
    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        IR_I2C_Abort();
        return 0U;
    }

    I2C_GenerateSTOP(IR_I2C_PERIPHERAL, ENABLE);
    return 1U;
}

static uint8_t IR_I2C_ReadRegister(uint8_t reg, uint8_t *value)
{
    if (value == 0)
    {
        return 0U;
    }

    if (!IR_I2C_WaitBusIdle())
    {
        return 0U;
    }

    /* 先以发送方向写入寄存器地址。 */
    I2C_GenerateSTART(IR_I2C_PERIPHERAL, ENABLE);
    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
    {
        IR_I2C_Abort();
        return 0U;
    }

    I2C_Send7bitAddress(IR_I2C_PERIPHERAL,
                        (uint8_t)(IR_I2C_ADDRESS_7BIT << 1),
                        I2C_Direction_Transmitter);
    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        IR_I2C_Abort();
        return 0U;
    }

    I2C_SendData(IR_I2C_PERIPHERAL, reg);
    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        IR_I2C_Abort();
        return 0U;
    }

    /* 重复起始，以接收方向读取一个字节。 */
    I2C_GenerateSTART(IR_I2C_PERIPHERAL, ENABLE);
    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
    {
        IR_I2C_Abort();
        return 0U;
    }

    /*
     * 单字节接收必须在清除 ADDR 前关闭 ACK。
     * I2C_CheckEvent(RECEIVER_MODE_SELECTED) 会读取 SR1/SR2 并清除 ADDR。
     */
    I2C_AcknowledgeConfig(IR_I2C_PERIPHERAL, DISABLE);
    I2C_Send7bitAddress(IR_I2C_PERIPHERAL,
                        (uint8_t)(IR_I2C_ADDRESS_7BIT << 1),
                        I2C_Direction_Receiver);

    if (!IR_I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        IR_I2C_Abort();
        return 0U;
    }

    I2C_GenerateSTOP(IR_I2C_PERIPHERAL, ENABLE);

    if (!IR_I2C_WaitRxNotEmpty())
    {
        IR_I2C_Abort();
        return 0U;
    }

    *value = I2C_ReceiveData(IR_I2C_PERIPHERAL);
    I2C_AcknowledgeConfig(IR_I2C_PERIPHERAL, ENABLE);
    return 1U;
}

void IR_I2C_Init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    gpio.GPIO_Pin = IR_I2C_SCL_PIN | IR_I2C_SDA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(IR_I2C_PORT, &gpio);

    I2C_DeInit(IR_I2C_PERIPHERAL);

    i2c.I2C_ClockSpeed = 400000U;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0U;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(IR_I2C_PERIPHERAL, &i2c);

    I2C_Cmd(IR_I2C_PERIPHERAL, ENABLE);
    I2C_AcknowledgeConfig(IR_I2C_PERIPHERAL, ENABLE);
}

uint8_t IR_I2C_ReadRaw(uint8_t *raw_data)
{
    return IR_I2C_ReadRegister(IR_REG_DIGITAL_DATA, raw_data);
}

uint8_t IR_I2C_SetAdjustMode(uint8_t enable)
{
    uint8_t mode = (enable != 0U) ? 1U : 0U;
    return IR_I2C_WriteRegister(IR_REG_ADJUST_MODE, mode);
}
