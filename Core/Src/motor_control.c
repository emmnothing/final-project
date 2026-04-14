#include "motor_control.h"

#include <math.h>

#include "app_config.h"
#include "app_platform.h"

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
} pid_axis_t;

typedef struct
{
  bool initialized;
  bool enabled;
  uint32_t pwm_arr;
  motor_driver_map_t map;
  pid_axis_t left_pid;
  pid_axis_t right_pid;
  wheel_setpoint_t setpoint;
} motor_control_ctx_t;

static motor_control_ctx_t g_motor_control;

static float motor_control_clampf(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float motor_control_pid_step(pid_axis_t *pid, float target, float measured, float dt_s)
{
  float derivative;
  float error;
  float output;

  error = target - measured;
  pid->integral += error * dt_s;
  pid->integral = motor_control_clampf(pid->integral, -2000.0f, 2000.0f);
  derivative = (dt_s > 0.0f) ? ((error - pid->prev_error) / dt_s) : 0.0f;
  pid->prev_error = error;

  output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  return motor_control_clampf(output, -1.0f, 1.0f);
}

static void motor_control_apply_channel_pair(uint32_t forward_channel, uint32_t reverse_channel, float signed_duty)
{
  TIM_HandleTypeDef *motor_tim = app_platform_motor_tim();
  const float magnitude = motor_control_clampf(fabsf(signed_duty), 0.0f, 1.0f);
  const uint32_t compare = (uint32_t)(magnitude * (float)g_motor_control.pwm_arr);

  if (signed_duty >= 0.0f)
  {
    __HAL_TIM_SET_COMPARE(motor_tim, forward_channel, compare);
    __HAL_TIM_SET_COMPARE(motor_tim, reverse_channel, 0U);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motor_tim, forward_channel, 0U);
    __HAL_TIM_SET_COMPARE(motor_tim, reverse_channel, compare);
  }
}

bool motor_control_init(void)
{
  const app_robot_config_t *cfg = app_config_get();

  g_motor_control.initialized = true;
  g_motor_control.enabled = false;
  g_motor_control.pwm_arr = __HAL_TIM_GET_AUTORELOAD(app_platform_motor_tim());
  g_motor_control.map.left.forward_channel = TIM_CHANNEL_1;
  g_motor_control.map.left.reverse_channel = TIM_CHANNEL_2;
  g_motor_control.map.right.forward_channel = TIM_CHANNEL_3;
  g_motor_control.map.right.reverse_channel = TIM_CHANNEL_4;
  g_motor_control.left_pid.kp = cfg->pid_kp;
  g_motor_control.left_pid.ki = cfg->pid_ki;
  g_motor_control.left_pid.kd = cfg->pid_kd;
  g_motor_control.right_pid.kp = cfg->pid_kp;
  g_motor_control.right_pid.ki = cfg->pid_ki;
  g_motor_control.right_pid.kd = cfg->pid_kd;
  g_motor_control.setpoint.timestamp_ms = HAL_GetTick();

  motor_control_safe_stop();
  return true;
}

void motor_control_enable(bool enabled)
{
  g_motor_control.enabled = enabled;
  if (!enabled)
  {
    motor_control_safe_stop();
  }
}

void motor_control_set_target_ticks_per_s(float left_ticks_per_s, float right_ticks_per_s)
{
  g_motor_control.setpoint.left_ticks_per_s = left_ticks_per_s;
  g_motor_control.setpoint.right_ticks_per_s = right_ticks_per_s;
  g_motor_control.setpoint.timestamp_ms = HAL_GetTick();
  app_state_set_wheel_setpoint(&g_motor_control.setpoint);
}

void motor_control_update(float dt_s, const wheel_state_t *feedback)
{
  float left_cmd;
  float right_cmd;

  if ((!g_motor_control.initialized) || (feedback == NULL))
  {
    return;
  }

  if (!g_motor_control.enabled)
  {
    motor_control_safe_stop();
    return;
  }

  left_cmd = motor_control_pid_step(&g_motor_control.left_pid,
                                    g_motor_control.setpoint.left_ticks_per_s,
                                    feedback->left_ticks_per_s,
                                    dt_s);
  right_cmd = motor_control_pid_step(&g_motor_control.right_pid,
                                     g_motor_control.setpoint.right_ticks_per_s,
                                     feedback->right_ticks_per_s,
                                     dt_s);

  motor_control_apply_channel_pair(g_motor_control.map.left.forward_channel,
                                   g_motor_control.map.left.reverse_channel,
                                   left_cmd);
  motor_control_apply_channel_pair(g_motor_control.map.right.forward_channel,
                                   g_motor_control.map.right.reverse_channel,
                                   right_cmd);
}

void motor_control_safe_stop(void)
{
  g_motor_control.left_pid.integral = 0.0f;
  g_motor_control.left_pid.prev_error = 0.0f;
  g_motor_control.right_pid.integral = 0.0f;
  g_motor_control.right_pid.prev_error = 0.0f;
  g_motor_control.setpoint.left_ticks_per_s = 0.0f;
  g_motor_control.setpoint.right_ticks_per_s = 0.0f;
  g_motor_control.setpoint.timestamp_ms = HAL_GetTick();

  motor_control_apply_channel_pair(g_motor_control.map.left.forward_channel,
                                   g_motor_control.map.left.reverse_channel,
                                   0.0f);
  motor_control_apply_channel_pair(g_motor_control.map.right.forward_channel,
                                   g_motor_control.map.right.reverse_channel,
                                   0.0f);

  app_state_set_wheel_setpoint(&g_motor_control.setpoint);
}

const motor_driver_map_t *motor_control_get_driver_map(void)
{
  return &g_motor_control.map;
}
