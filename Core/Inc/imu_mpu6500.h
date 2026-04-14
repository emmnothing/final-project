#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "app_state.h"

bool imu_mpu6500_init(void);
imu_state_t imu_mpu6500_poll(void);

#ifdef __cplusplus
}
#endif

#endif
