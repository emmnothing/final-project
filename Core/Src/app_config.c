#include "app_config.h"

static const app_robot_config_t g_robot_config =
{
  .wheel_radius_m = 0.033f,
  .wheel_base_m = 0.160f,
  .encoder_ticks_per_rev = 800.0f,
  .default_speed_limit_mps = 0.20f,
  .default_turn_limit_radps = 1.50f,
  .default_safety_distance_m = 0.25f,
  .max_speed_limit_mps = 0.60f,
  .max_turn_limit_radps = 4.00f,
  .max_safety_distance_m = 0.80f,
  .pid_kp = 0.020f,
  .pid_ki = 0.080f,
  .pid_kd = 0.0005f,
  .oled_i2c_addr_7bit = 0x3CU,
  .imu_i2c_addr_7bit = 0x68U,
};

static float app_config_map_adc(uint16_t adc_raw, float min_value, float max_value)
{
  const float ratio = (float)adc_raw / 4095.0f;
  return min_value + ((max_value - min_value) * ratio);
}

const app_robot_config_t *app_config_get(void)
{
  return &g_robot_config;
}

float app_config_map_speed_limit(uint16_t adc_raw)
{
  return app_config_map_adc(adc_raw, 0.05f, g_robot_config.max_speed_limit_mps);
}

float app_config_map_turn_limit(uint16_t adc_raw)
{
  return app_config_map_adc(adc_raw, 0.20f, g_robot_config.max_turn_limit_radps);
}

float app_config_map_safety_distance(uint16_t adc_raw)
{
  return app_config_map_adc(adc_raw, 0.10f, g_robot_config.max_safety_distance_m);
}
