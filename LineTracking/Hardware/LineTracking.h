#ifndef __LINE_TRACKING_H
#define __LINE_TRACKING_H

#include "stm32f10x.h"

typedef enum
{
    LINE_TRACK_STOPPED = 0,
    LINE_TRACK_RUNNING,
    LINE_TRACK_LINE_LOST,
    LINE_TRACK_I2C_FAULT
} LineTrackingState;

void LineTracking_Init(float base_speed_cms);
void LineTracking_Start(void);
void LineTracking_SetBaseSpeed(float base_speed_cms);
void LineTracking_Stop(void);

/* 每 10ms 调用一次。raw_data 中 0 表示检测到黑线。 */
void LineTracking_Update(uint8_t raw_data);

/* 每次 I2C 读取失败时调用；连续失败会主动停车。 */
void LineTracking_OnI2CFailure(void);

LineTrackingState LineTracking_GetState(void);
float LineTracking_GetError(void);
float LineTracking_GetCorrectionCMS(void);
uint8_t LineTracking_GetRawData(void);
uint8_t LineTracking_GetActiveCount(void);

/*
 * 横向宽线/启停线事件。
 * GetMarkerEvent() 读取后自动清除单次事件；
 * GetMarkerCount() 返回累计次数。
 */
uint8_t LineTracking_GetMarkerEvent(void);
uint32_t LineTracking_GetMarkerCount(void);

#endif
