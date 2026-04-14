#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "app_state.h"

bool encoder_init(void);
void encoder_reset(void);
wheel_state_t encoder_sample(float dt_s);

#ifdef __cplusplus
}
#endif

#endif
