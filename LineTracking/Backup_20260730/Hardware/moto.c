#include "moto.h"
#include "pwm.h"

void moto(int mode)
{
    if (mode == 1)
    {
        PWMA_IN1 = 7200;
        PWMA_IN2 = 5000;
        PWMB_IN1 = 7200;
        PWMB_IN2 = 5000;
    }
    else if (mode == 0)
    {
        PWMA_IN2 = 7200;
        PWMA_IN1 = 5000;
        PWMB_IN2 = 7200;
        PWMB_IN1 = 5000;
    }
}

int Velocity_A(int Target_Vel, int Current_Vel)
{
    static float Encoder_bias = 0.0f;
    static float Encoder_Integral = 0.0f;
    float Encoder_Least;
    float FeedForward;
    float velocity;

    /*
     * Set_PWM(0, 0) 会让两个输入同时保持高电平。
     * 对当前驱动方式而言这是主动制动状态。
     *
     * 必须直接返回，不能像原程序一样清零后又继续根据 Current_Vel
     * 重新产生反向 PI 输出，否则停车时可能短暂反拖或来回修正。
     */
    if (Target_Vel == 0)
    {
        Encoder_bias = 0.0f;
        Encoder_Integral = 0.0f;
        return 0;
    }

    if (Target_Vel > 0)
    {
        FeedForward = 320.0f + Target_Vel * 86.0f;
    }
    else
    {
        FeedForward = -320.0f + Target_Vel * 86.0f;
    }

    Encoder_Least = (float)(Target_Vel - Current_Vel);
    Encoder_bias = Encoder_bias * 0.84f + Encoder_Least * 0.16f;
    Encoder_Integral += Encoder_bias;

    if (Encoder_Integral > 14000.0f)
    {
        Encoder_Integral = 14000.0f;
    }
    if (Encoder_Integral < -14000.0f)
    {
        Encoder_Integral = -14000.0f;
    }

    velocity = FeedForward
             + Encoder_bias * 20.0f
             + Encoder_Integral * 0.5f;

    if (velocity > 7199.0f)
    {
        velocity = 7199.0f;
    }
    if (velocity < -7199.0f)
    {
        velocity = -7199.0f;
    }

    return (int)velocity;
}

int Velocity_B(int Target_Vel, int Current_Vel)
{
    static float Encoder_bias = 0.0f;
    static float Encoder_Integral = 0.0f;
    float Encoder_Least;
    float FeedForward;
    float velocity;

    if (Target_Vel == 0)
    {
        Encoder_bias = 0.0f;
        Encoder_Integral = 0.0f;
        return 0;
    }

    if (Target_Vel > 0)
    {
        FeedForward = 360.0f + Target_Vel * 92.4f;
    }
    else
    {
        FeedForward = -360.0f + Target_Vel * 92.4f;
    }

    Encoder_Least = (float)(Target_Vel - Current_Vel);
    Encoder_bias = Encoder_bias * 0.84f + Encoder_Least * 0.16f;
    Encoder_Integral += Encoder_bias;

    if (Encoder_Integral > 14000.0f)
    {
        Encoder_Integral = 14000.0f;
    }
    if (Encoder_Integral < -14000.0f)
    {
        Encoder_Integral = -14000.0f;
    }

    velocity = FeedForward
             + Encoder_bias * 20.0f
             + Encoder_Integral * 0.5f;

    if (velocity > 7199.0f)
    {
        velocity = 7199.0f;
    }
    if (velocity < -7199.0f)
    {
        velocity = -7199.0f;
    }

    return (int)velocity;
}
