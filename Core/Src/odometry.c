#include "odometry.h"

#include <math.h>

#include "app_config.h"
#include "stm32f4xx_hal.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static pose2d_t g_pose;
static uint32_t g_last_wheel_timestamp_ms;

static float odometry_wrap_angle(float angle)
{
  while (angle > (float)M_PI)
  {
    angle -= (2.0f * (float)M_PI);
  }
  while (angle < -(float)M_PI)
  {
    angle += (2.0f * (float)M_PI);
  }
  return angle;
}

void odometry_init(void)
{
  odometry_reset();
}

void odometry_reset(void)
{
  g_pose.x_m = 0.0f;
  g_pose.y_m = 0.0f;
  g_pose.yaw_rad = 0.0f;
  g_pose.timestamp_ms = HAL_GetTick();
  g_last_wheel_timestamp_ms = 0U;
}

pose2d_t odometry_update(const wheel_state_t *wheel_state, const imu_state_t *imu_state, float dt_s)
{
  const app_robot_config_t *cfg;
  float meters_per_tick;
  float dl;
  float dr;
  float ds;
  float dyaw_encoder;
  float dyaw;
  float heading_mid;

  if (wheel_state == NULL)
  {
    return g_pose;
  }

  if (wheel_state->timestamp_ms == g_last_wheel_timestamp_ms)
  {
    return g_pose;
  }

  g_last_wheel_timestamp_ms = wheel_state->timestamp_ms;
  cfg = app_config_get();
  meters_per_tick = (2.0f * (float)M_PI * cfg->wheel_radius_m) / cfg->encoder_ticks_per_rev;
  dl = (float)wheel_state->left_delta_ticks * meters_per_tick;
  dr = (float)wheel_state->right_delta_ticks * meters_per_tick;
  ds = 0.5f * (dl + dr);
  dyaw_encoder = (dr - dl) / cfg->wheel_base_m;
  dyaw = dyaw_encoder;

  if ((imu_state != NULL) && imu_state->initialized && imu_state->healthy)
  {
    dyaw = (0.70f * dyaw_encoder) + (0.30f * imu_state->gyro_z_radps * dt_s);
  }

  heading_mid = g_pose.yaw_rad + (0.5f * dyaw);
  g_pose.x_m += ds * cosf(heading_mid);
  g_pose.y_m += ds * sinf(heading_mid);
  g_pose.yaw_rad = odometry_wrap_angle(g_pose.yaw_rad + dyaw);
  g_pose.timestamp_ms = wheel_state->timestamp_ms;

  return g_pose;
}
