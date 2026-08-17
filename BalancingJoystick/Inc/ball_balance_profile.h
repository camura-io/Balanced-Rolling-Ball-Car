#ifndef BALL_BALANCE_PROFILE_H
#define BALL_BALANCE_PROFILE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t BallBalance_StaticSteps(int32_t pos_mm, int32_t steps_per_rev);

#ifdef __cplusplus
}
#endif

#endif
