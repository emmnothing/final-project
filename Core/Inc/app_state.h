#ifndef APP_STATE_H
#define APP_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
  APP_MODE_IDLE = 0,
  APP_MODE_RUN,
  APP_MODE_RETURN,
  APP_MODE_STOP,
  APP_MODE_FAULT,
} app_mode_t;

typedef enum
{
  APP_COMMAND_NONE = 0,
  APP_COMMAND_START,
  APP_COMMAND_STOP,
  APP_COMMAND_RETURN,
  APP_COMMAND_PAGE,
  APP_COMMAND_CLEAR_FAULT,
  APP_COMMAND_RESET_ODOM,
} app_command_t;

typedef enum
{
  APP_PARAM_SPEED_MAX = 0,
  APP_PARAM_TURN_MAX,
  APP_PARAM_SAFETY_DISTANCE,
} app_param_id_t;

typedef enum
{
  APP_FAULT_NONE            = 0U,
  APP_FAULT_PWM_START       = (1UL << 0),
  APP_FAULT_ENCODER_START   = (1UL << 1),
  APP_FAULT_ADC_START       = (1UL << 2),
  APP_FAULT_LIDAR_UART      = (1UL << 3),
  APP_FAULT_BT_UART         = (1UL << 4),
  APP_FAULT_OLED_INIT       = (1UL << 5),
  APP_FAULT_IMU_INIT        = (1UL << 6),
  APP_FAULT_ENCODER_STALE   = (1UL << 7),
  APP_FAULT_CONTROL_SAT     = (1UL << 8),
  APP_FAULT_I2C_BUS         = (1UL << 9),
} fault_code_t;

#define APP_FATAL_FAULT_MASK (APP_FAULT_PWM_START | APP_FAULT_ENCODER_START | APP_FAULT_ADC_START | \
                              APP_FAULT_ENCODER_STALE | APP_FAULT_CONTROL_SAT)

typedef enum
{
  BUTTON_EVENT_NONE = 0,
  BUTTON_EVENT_START,
  BUTTON_EVENT_STOP,
  BUTTON_EVENT_RETURN,
  BUTTON_EVENT_PAGE,
} button_event_t;

typedef struct
{
  float left_ticks_per_s;
  float right_ticks_per_s;
  uint32_t timestamp_ms;
} wheel_setpoint_t;

typedef struct
{
  int32_t left_total_ticks;
  int32_t right_total_ticks;
  int32_t left_delta_ticks;
  int32_t right_delta_ticks;
  float left_ticks_per_s;
  float right_ticks_per_s;
  uint32_t timestamp_ms;
} wheel_state_t;

typedef struct
{
  float x_m;
  float y_m;
  float yaw_rad;
  uint32_t timestamp_ms;
} pose2d_t;

typedef struct
{
  float speed_max_mps;
  float turn_max_radps;
  float safety_distance_m;
  uint16_t raw_adc[3];
} pot_params_t;

typedef struct
{
  bool speed_max_valid;
  bool turn_max_valid;
  bool safety_distance_valid;
  float speed_max_mps;
  float turn_max_radps;
  float safety_distance_m;
} app_param_override_t;

typedef struct
{
  uint32_t bytes_received;
  uint32_t frames_parsed;
  uint16_t last_sample_count;
  uint32_t last_frame_ms;
  bool stream_active;
  bool parser_locked;
} lidar_frame_t;

typedef struct
{
  bool initialized;
  bool healthy;
  uint8_t who_am_i;
  float gyro_z_radps;
  uint32_t timestamp_ms;
} imu_state_t;

typedef struct
{
  app_mode_t mode;
  uint32_t faults;
  uint8_t oled_page;
  bool bt_link_active;
  bool lidar_ready;
  pot_params_t params;
  wheel_setpoint_t setpoint;
  wheel_state_t wheels;
  pose2d_t pose;
  lidar_frame_t lidar;
  imu_state_t imu;
  app_param_override_t overrides;
} app_snapshot_t;

bool app_state_init(void);
bool app_state_submit_command(app_command_t command);
bool app_state_wait_command(app_command_t *command, TickType_t timeout_ticks);

void app_state_set_mode(app_mode_t mode);
app_mode_t app_state_get_mode(void);

void app_state_set_fault(uint32_t fault_mask);
void app_state_clear_fault(uint32_t fault_mask);
uint32_t app_state_get_faults(void);
bool app_state_has_fatal_fault(void);
bool app_state_can_start(void);

void app_state_cycle_page(void);

void app_state_set_bt_link(bool active);
void app_state_set_lidar_ready(bool ready);

pot_params_t app_state_apply_param_overrides(const pot_params_t *measured);
bool app_state_set_param_override(app_param_id_t id, float value);
void app_state_clear_param_overrides(void);

void app_state_set_wheel_setpoint(const wheel_setpoint_t *setpoint);
wheel_setpoint_t app_state_get_wheel_setpoint(void);

void app_state_set_pot_params(const pot_params_t *params);
void app_state_set_wheel_state(const wheel_state_t *state);
void app_state_set_pose(const pose2d_t *pose);
void app_state_set_lidar_frame(const lidar_frame_t *frame);
void app_state_set_imu_state(const imu_state_t *imu);

app_snapshot_t app_state_get_snapshot(void);
const char *app_state_mode_string(app_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif
