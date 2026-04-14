#ifndef SAFETY_H
#define SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

void safety_init(void);
void safety_safe_stop(void);
void safety_handle_fault(uint32_t fault_mask);
void safety_periodic_check(const wheel_state_t *wheel_state);

#ifdef __cplusplus
}
#endif

#endif
