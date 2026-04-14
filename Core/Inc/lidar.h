#ifndef LIDAR_H
#define LIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "app_state.h"

bool lidar_init(void);
void lidar_process_dma(void);
lidar_frame_t lidar_get_latest(void);
bool lidar_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif
