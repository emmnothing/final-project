#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define APP_ADC_CHANNEL_COUNT          3U
#define APP_LIDAR_DMA_BUFFER_SIZE      1024U
#define APP_BT_RX_RING_SIZE            256U
#define APP_BT_LINE_BUFFER_SIZE        96U
#define APP_OLED_WIDTH                 128U
#define APP_OLED_HEIGHT                64U
#define APP_OLED_BUFFER_SIZE           (APP_OLED_WIDTH * APP_OLED_HEIGHT / 8U)

#define APP_SUPERVISOR_PERIOD_MS       20U
#define APP_CONTROL_PERIOD_MS          20U
#define APP_PERCEPTION_PERIOD_MS       10U
#define APP_UI_PERIOD_MS               100U
#define APP_TELEMETRY_PERIOD_MS        250U

#define APP_LIDAR_STREAM_TIMEOUT_MS    500U
#define APP_ENCODER_STALE_TIMEOUT_MS   250U

typedef struct
{
  float wheel_radius_m;
  float wheel_base_m;
  float encoder_ticks_per_rev;
  float default_speed_limit_mps;
  float default_turn_limit_radps;
  float default_safety_distance_m;
  float max_speed_limit_mps;
  float max_turn_limit_radps;
  float max_safety_distance_m;
  float pid_kp;
  float pid_ki;
  float pid_kd;
  uint8_t oled_i2c_addr_7bit;
  uint8_t imu_i2c_addr_7bit;
} app_robot_config_t;

const app_robot_config_t *app_config_get(void);

float app_config_map_speed_limit(uint16_t adc_raw);
float app_config_map_turn_limit(uint16_t adc_raw);
float app_config_map_safety_distance(uint16_t adc_raw);

#ifdef __cplusplus
}
#endif

#endif
