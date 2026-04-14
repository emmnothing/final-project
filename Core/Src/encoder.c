#include "encoder.h"

#include "app_platform.h"

typedef struct
{
  int32_t left_prev;
  int32_t right_prev;
  int32_t left_total;
  int32_t right_total;
  bool initialized;
} encoder_ctx_t;

static encoder_ctx_t g_encoder;

static int32_t encoder_read_counter(TIM_HandleTypeDef *htim)
{
  return (int32_t)__HAL_TIM_GET_COUNTER(htim);
}

static int32_t encoder_unwrap_delta(int32_t current, int32_t previous)
{
  int32_t delta = current - previous;

  if (delta > 32767)
  {
    delta -= 65536;
  }
  else if (delta < -32768)
  {
    delta += 65536;
  }

  return delta;
}

bool encoder_init(void)
{
  g_encoder.left_prev = encoder_read_counter(app_platform_left_encoder_tim());
  g_encoder.right_prev = encoder_read_counter(app_platform_right_encoder_tim());
  g_encoder.left_total = 0;
  g_encoder.right_total = 0;
  g_encoder.initialized = true;
  return true;
}

void encoder_reset(void)
{
  (void)encoder_init();
}

wheel_state_t encoder_sample(float dt_s)
{
  wheel_state_t state = {0};
  int32_t left_now;
  int32_t right_now;
  int32_t left_delta;
  int32_t right_delta;

  if (!g_encoder.initialized)
  {
    (void)encoder_init();
  }

  left_now = encoder_read_counter(app_platform_left_encoder_tim());
  right_now = encoder_read_counter(app_platform_right_encoder_tim());

  left_delta = encoder_unwrap_delta(left_now, g_encoder.left_prev);
  right_delta = encoder_unwrap_delta(right_now, g_encoder.right_prev);

  g_encoder.left_prev = left_now;
  g_encoder.right_prev = right_now;
  g_encoder.left_total += left_delta;
  g_encoder.right_total += right_delta;

  state.left_total_ticks = g_encoder.left_total;
  state.right_total_ticks = g_encoder.right_total;
  state.left_delta_ticks = left_delta;
  state.right_delta_ticks = right_delta;
  state.timestamp_ms = HAL_GetTick();

  if (dt_s > 0.0f)
  {
    state.left_ticks_per_s = (float)left_delta / dt_s;
    state.right_ticks_per_s = (float)right_delta / dt_s;
  }

  return state;
}
