#ifndef APP_PLATFORM_H
#define APP_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"

bool app_platform_init(void);

bool app_platform_start_pwm(void);
bool app_platform_start_encoders(void);
bool app_platform_start_adc_dma(void);
bool app_platform_start_lidar_dma(void);

uint16_t *app_platform_get_adc_buffer(void);
uint8_t *app_platform_get_lidar_dma_buffer(void);
size_t app_platform_get_lidar_dma_buffer_size(void);

TIM_HandleTypeDef *app_platform_motor_tim(void);
TIM_HandleTypeDef *app_platform_left_encoder_tim(void);
TIM_HandleTypeDef *app_platform_right_encoder_tim(void);
UART_HandleTypeDef *app_platform_lidar_uart(void);
UART_HandleTypeDef *app_platform_bt_uart(void);
I2C_HandleTypeDef *app_platform_i2c(void);

bool app_platform_i2c_acquire(TickType_t timeout_ticks);
void app_platform_i2c_release(void);

void app_platform_status_led_write(bool on);
void app_platform_status_led_toggle(void);

#ifdef __cplusplus
}
#endif

#endif
