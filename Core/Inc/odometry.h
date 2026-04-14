#ifndef ODOMETRY_H
#define ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

void odometry_init(void);
void odometry_reset(void);
pose2d_t odometry_update(const wheel_state_t *wheel_state, const imu_state_t *imu_state, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
