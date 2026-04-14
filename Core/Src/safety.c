#include "safety.h"

#include "app_config.h"
#include "app_state.h"
#include "motor_control.h"
#include "stm32f4xx_hal.h"

static uint32_t g_last_encoder_ms;

void safety_init(void)
{
  g_last_encoder_ms = HAL_GetTick();
}

void safety_safe_stop(void)
{
  motor_control_enable(false);
  motor_control_safe_stop();
}

void safety_handle_fault(uint32_t fault_mask)
{
  app_state_set_fault(fault_mask);
  if ((fault_mask & APP_FATAL_FAULT_MASK) != 0U)
  {
    safety_safe_stop();
  }
}

void safety_periodic_check(const wheel_state_t *wheel_state)
{
  const app_mode_t mode = app_state_get_mode();
  const uint32_t now = HAL_GetTick();

  if (wheel_state != NULL)
  {
    g_last_encoder_ms = wheel_state->timestamp_ms;
  }

  if (((mode == APP_MODE_RUN) || (mode == APP_MODE_RETURN)) &&
      ((now - g_last_encoder_ms) > APP_ENCODER_STALE_TIMEOUT_MS))
  {
    safety_handle_fault(APP_FAULT_ENCODER_STALE);
  }
}
