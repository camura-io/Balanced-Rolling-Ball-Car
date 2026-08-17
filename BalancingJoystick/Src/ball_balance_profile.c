#include "ball_balance_profile.h"

#define BALL_BALANCE_POS_NEG10_MM  -100L /* 球在-10cm时，电机静态平衡角约+2.8度。 */
#define BALL_BALANCE_POS_NEG5_MM   -50L
#define BALL_BALANCE_POS_ZERO_MM   0L
#define BALL_BALANCE_POS_POS5_MM   50L
#define BALL_BALANCE_POS_POS10_MM  100L
#define BALL_BALANCE_ANG_NEG10_D10 28L
#define BALL_BALANCE_ANG_NEG5_D10  10L
#define BALL_BALANCE_ANG_ZERO_D10  0L
#define BALL_BALANCE_ANG_POS5_D10  -28L
#define BALL_BALANCE_ANG_POS10_D10 -56L

static int32_t BallBalance_AngleD10ToSteps(int32_t angle_d10, int32_t steps_per_rev)
{
  int32_t numerator;

  numerator = angle_d10 * steps_per_rev;
  if (numerator >= 0L)
  {
    return (numerator + 1800L) / 3600L;
  }
  return -((-numerator + 1800L) / 3600L);
}

static int32_t BallBalance_InterpolateAngleD10(int32_t pos_mm,
                                               int32_t pos0_mm, int32_t angle0_d10,
                                               int32_t pos1_mm, int32_t angle1_d10)
{
  int32_t numerator;
  int32_t denominator;

  denominator = pos1_mm - pos0_mm;
  if (denominator == 0L)
  {
    return angle0_d10;
  }

  numerator = (pos_mm - pos0_mm) * (angle1_d10 - angle0_d10);
  if (numerator >= 0L)
  {
    return angle0_d10 + ((numerator + (denominator / 2L)) / denominator);
  }
  return angle0_d10 + ((numerator - (denominator / 2L)) / denominator);
}

int32_t BallBalance_StaticSteps(int32_t pos_mm, int32_t steps_per_rev)
{
  int32_t angle_d10;

  /* 位置到静态平衡角：负方向位置需要正角度，正方向位置需要负角度。 */
  if (pos_mm <= BALL_BALANCE_POS_NEG5_MM)
  {
    angle_d10 = BallBalance_InterpolateAngleD10(pos_mm,
                                                BALL_BALANCE_POS_NEG10_MM,
                                                BALL_BALANCE_ANG_NEG10_D10,
                                                BALL_BALANCE_POS_NEG5_MM,
                                                BALL_BALANCE_ANG_NEG5_D10);
  }
  else if (pos_mm <= BALL_BALANCE_POS_ZERO_MM)
  {
    angle_d10 = BallBalance_InterpolateAngleD10(pos_mm,
                                                BALL_BALANCE_POS_NEG5_MM,
                                                BALL_BALANCE_ANG_NEG5_D10,
                                                BALL_BALANCE_POS_ZERO_MM,
                                                BALL_BALANCE_ANG_ZERO_D10);
  }
  else if (pos_mm <= BALL_BALANCE_POS_POS5_MM)
  {
    angle_d10 = BallBalance_InterpolateAngleD10(pos_mm,
                                                BALL_BALANCE_POS_ZERO_MM,
                                                BALL_BALANCE_ANG_ZERO_D10,
                                                BALL_BALANCE_POS_POS5_MM,
                                                BALL_BALANCE_ANG_POS5_D10);
  }
  else
  {
    angle_d10 = BallBalance_InterpolateAngleD10(pos_mm,
                                                BALL_BALANCE_POS_POS5_MM,
                                                BALL_BALANCE_ANG_POS5_D10,
                                                BALL_BALANCE_POS_POS10_MM,
                                                BALL_BALANCE_ANG_POS10_D10);
  }

  return BallBalance_AngleD10ToSteps(angle_d10, steps_per_rev);
}
