#ifndef UI_OLED_H
#define UI_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool ui_oled_init(void);
void ui_oled_render(void);

#ifdef __cplusplus
}
#endif

#endif
