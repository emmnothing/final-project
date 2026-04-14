#ifndef TELEMETRY_BT_H
#define TELEMETRY_BT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

bool telemetry_bt_init(void);
void telemetry_bt_process_rx(void);
void telemetry_bt_send_periodic(void);
void telemetry_bt_on_rx_complete(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
