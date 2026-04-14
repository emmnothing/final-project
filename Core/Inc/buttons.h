#ifndef BUTTONS_H
#define BUTTONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_state.h"

void buttons_init(void);
button_event_t buttons_poll(void);

#ifdef __cplusplus
}
#endif

#endif
