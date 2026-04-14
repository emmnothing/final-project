#include "app_state.h"

#include <string.h>

#include "queue.h"
#include "semphr.h"

typedef struct
{
  QueueHandle_t command_queue;
  SemaphoreHandle_t mutex;
  app_snapshot_t snapshot;
} app_state_ctx_t;

static app_state_ctx_t g_app_state;

static void app_state_lock(void)
{
  if (g_app_state.mutex != NULL)
  {
    (void)xSemaphoreTake(g_app_state.mutex, portMAX_DELAY);
  }
}

static void app_state_unlock(void)
{
  if (g_app_state.mutex != NULL)
  {
    (void)xSemaphoreGive(g_app_state.mutex);
  }
}

bool app_state_init(void)
{
  memset(&g_app_state, 0, sizeof(g_app_state));
  g_app_state.command_queue = xQueueCreate(16U, sizeof(app_command_t));
  g_app_state.mutex = xSemaphoreCreateMutex();
  if ((g_app_state.command_queue == NULL) || (g_app_state.mutex == NULL))
  {
    return false;
  }

  g_app_state.snapshot.mode = APP_MODE_STOP;
  g_app_state.snapshot.lidar_ready = false;
  return true;
}

bool app_state_submit_command(app_command_t command)
{
  if (g_app_state.command_queue == NULL)
  {
    return false;
  }
  return (xQueueSend(g_app_state.command_queue, &command, 0U) == pdPASS);
}

bool app_state_wait_command(app_command_t *command, TickType_t timeout_ticks)
{
  if ((g_app_state.command_queue == NULL) || (command == NULL))
  {
    return false;
  }
  return (xQueueReceive(g_app_state.command_queue, command, timeout_ticks) == pdPASS);
}

void app_state_set_mode(app_mode_t mode)
{
  app_state_lock();
  g_app_state.snapshot.mode = mode;
  app_state_unlock();
}

app_mode_t app_state_get_mode(void)
{
  app_mode_t mode;

  app_state_lock();
  mode = g_app_state.snapshot.mode;
  app_state_unlock();

  return mode;
}

void app_state_set_fault(uint32_t fault_mask)
{
  app_state_lock();
  g_app_state.snapshot.faults |= fault_mask;
  if ((g_app_state.snapshot.faults & APP_FATAL_FAULT_MASK) != 0U)
  {
    g_app_state.snapshot.mode = APP_MODE_FAULT;
  }
  app_state_unlock();
}

void app_state_clear_fault(uint32_t fault_mask)
{
  app_state_lock();
  g_app_state.snapshot.faults &= ~fault_mask;
  if ((g_app_state.snapshot.mode == APP_MODE_FAULT) &&
      ((g_app_state.snapshot.faults & APP_FATAL_FAULT_MASK) == 0U))
  {
    g_app_state.snapshot.mode = APP_MODE_STOP;
  }
  app_state_unlock();
}

uint32_t app_state_get_faults(void)
{
  uint32_t faults;

  app_state_lock();
  faults = g_app_state.snapshot.faults;
  app_state_unlock();

  return faults;
}

bool app_state_has_fatal_fault(void)
{
  return ((app_state_get_faults() & APP_FATAL_FAULT_MASK) != 0U);
}

bool app_state_can_start(void)
{
  bool can_start;

  app_state_lock();
  can_start = ((g_app_state.snapshot.faults & APP_FATAL_FAULT_MASK) == 0U);
  app_state_unlock();

  return can_start;
}

void app_state_cycle_page(void)
{
  app_state_lock();
  g_app_state.snapshot.oled_page = (uint8_t)((g_app_state.snapshot.oled_page + 1U) % 2U);
  app_state_unlock();
}

void app_state_set_bt_link(bool active)
{
  app_state_lock();
  g_app_state.snapshot.bt_link_active = active;
  app_state_unlock();
}

void app_state_set_lidar_ready(bool ready)
{
  app_state_lock();
  g_app_state.snapshot.lidar_ready = ready;
  app_state_unlock();
}

pot_params_t app_state_apply_param_overrides(const pot_params_t *measured)
{
  pot_params_t effective = {0};

  if (measured != NULL)
  {
    effective = *measured;
  }

  app_state_lock();
  if (g_app_state.snapshot.overrides.speed_max_valid)
  {
    effective.speed_max_mps = g_app_state.snapshot.overrides.speed_max_mps;
  }
  if (g_app_state.snapshot.overrides.turn_max_valid)
  {
    effective.turn_max_radps = g_app_state.snapshot.overrides.turn_max_radps;
  }
  if (g_app_state.snapshot.overrides.safety_distance_valid)
  {
    effective.safety_distance_m = g_app_state.snapshot.overrides.safety_distance_m;
  }
  g_app_state.snapshot.params = effective;
  app_state_unlock();

  return effective;
}

bool app_state_set_param_override(app_param_id_t id, float value)
{
  bool accepted = true;

  app_state_lock();
  switch (id)
  {
    case APP_PARAM_SPEED_MAX:
      g_app_state.snapshot.overrides.speed_max_valid = true;
      g_app_state.snapshot.overrides.speed_max_mps = value;
      break;
    case APP_PARAM_TURN_MAX:
      g_app_state.snapshot.overrides.turn_max_valid = true;
      g_app_state.snapshot.overrides.turn_max_radps = value;
      break;
    case APP_PARAM_SAFETY_DISTANCE:
      g_app_state.snapshot.overrides.safety_distance_valid = true;
      g_app_state.snapshot.overrides.safety_distance_m = value;
      break;
    default:
      accepted = false;
      break;
  }
  app_state_unlock();

  return accepted;
}

void app_state_clear_param_overrides(void)
{
  app_state_lock();
  memset(&g_app_state.snapshot.overrides, 0, sizeof(g_app_state.snapshot.overrides));
  app_state_unlock();
}

void app_state_set_wheel_setpoint(const wheel_setpoint_t *setpoint)
{
  if (setpoint == NULL)
  {
    return;
  }

  app_state_lock();
  g_app_state.snapshot.setpoint = *setpoint;
  app_state_unlock();
}

wheel_setpoint_t app_state_get_wheel_setpoint(void)
{
  wheel_setpoint_t setpoint = {0};

  app_state_lock();
  setpoint = g_app_state.snapshot.setpoint;
  app_state_unlock();

  return setpoint;
}

void app_state_set_pot_params(const pot_params_t *params)
{
  if (params == NULL)
  {
    return;
  }

  app_state_lock();
  g_app_state.snapshot.params = *params;
  app_state_unlock();
}

void app_state_set_wheel_state(const wheel_state_t *state)
{
  if (state == NULL)
  {
    return;
  }

  app_state_lock();
  g_app_state.snapshot.wheels = *state;
  app_state_unlock();
}

void app_state_set_pose(const pose2d_t *pose)
{
  if (pose == NULL)
  {
    return;
  }

  app_state_lock();
  g_app_state.snapshot.pose = *pose;
  app_state_unlock();
}

void app_state_set_lidar_frame(const lidar_frame_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  app_state_lock();
  g_app_state.snapshot.lidar = *frame;
  g_app_state.snapshot.lidar_ready = frame->stream_active;
  app_state_unlock();
}

void app_state_set_imu_state(const imu_state_t *imu)
{
  if (imu == NULL)
  {
    return;
  }

  app_state_lock();
  g_app_state.snapshot.imu = *imu;
  app_state_unlock();
}

app_snapshot_t app_state_get_snapshot(void)
{
  app_snapshot_t snapshot;

  app_state_lock();
  snapshot = g_app_state.snapshot;
  app_state_unlock();

  return snapshot;
}

const char *app_state_mode_string(app_mode_t mode)
{
  switch (mode)
  {
    case APP_MODE_IDLE:
      return "IDLE";
    case APP_MODE_RUN:
      return "RUN";
    case APP_MODE_RETURN:
      return "RETURN";
    case APP_MODE_STOP:
      return "STOP";
    case APP_MODE_FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}
