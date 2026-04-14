#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "app_state.h"

typedef struct
{
  uint32_t forward_channel;
  uint32_t reverse_channel;
} motor_channel_map_t;

typedef struct
{
  motor_channel_map_t left;
  motor_channel_map_t right;
} motor_driver_map_t;

bool motor_control_init(void);
void motor_control_enable(bool enabled);
void motor_control_set_target_ticks_per_s(float left_ticks_per_s, float right_ticks_per_s);
void motor_control_update(float dt_s, const wheel_state_t *feedback);
void motor_control_safe_stop(void);
const motor_driver_map_t *motor_control_get_driver_map(void);

#ifdef __cplusplus
}
#endif

#endif
