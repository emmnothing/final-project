#include "app_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_platform.h"
#include "app_state.h"
#include "buttons.h"
#include "encoder.h"
#include "imu_mpu6500.h"
#include "lidar.h"
#include "motor_control.h"
#include "odometry.h"
#include "safety.h"
#include "telemetry_bt.h"
#include "ui_oled.h"

static TaskHandle_t g_supervisor_task;
static TaskHandle_t g_control_task;
static TaskHandle_t g_perception_task;
static TaskHandle_t g_ui_task;

static pot_params_t app_tasks_read_pots(void)
{
  pot_params_t params = {0};
  const uint16_t *adc = app_platform_get_adc_buffer();

  params.raw_adc[0] = adc[0];
  params.raw_adc[1] = adc[1];
  params.raw_adc[2] = adc[2];
  params.speed_max_mps = app_config_map_speed_limit(adc[0]);
  params.turn_max_radps = app_config_map_turn_limit(adc[1]);
  params.safety_distance_m = app_config_map_safety_distance(adc[2]);
  return params;
}

static void app_tasks_apply_command(app_command_t command)
{
  switch (command)
  {
    case APP_COMMAND_START:
      if (app_state_can_start())
      {
        app_state_set_mode(lidar_is_ready() ? APP_MODE_RUN : APP_MODE_IDLE);
      }
      break;
    case APP_COMMAND_STOP:
      app_state_set_mode(APP_MODE_STOP);
      safety_safe_stop();
      break;
    case APP_COMMAND_RETURN:
      if (app_state_can_start())
      {
        app_state_set_mode(APP_MODE_RETURN);
      }
      break;
    case APP_COMMAND_PAGE:
      app_state_cycle_page();
      break;
    case APP_COMMAND_CLEAR_FAULT:
      app_state_clear_fault(APP_FATAL_FAULT_MASK | APP_FAULT_BT_UART | APP_FAULT_OLED_INIT | APP_FAULT_IMU_INIT |
                            APP_FAULT_LIDAR_UART | APP_FAULT_I2C_BUS | APP_FAULT_CONTROL_SAT);
      break;
    case APP_COMMAND_RESET_ODOM:
      odometry_reset();
      break;
    case APP_COMMAND_NONE:
    default:
      break;
  }
}

static void app_task_supervisor(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();

  (void)argument;
  for (;;)
  {
    app_command_t command;
    const button_event_t button_event = buttons_poll();
    pot_params_t measured = app_tasks_read_pots();
    pot_params_t effective = app_state_apply_param_overrides(&measured);
    app_mode_t mode;

    if (button_event == BUTTON_EVENT_START)
    {
      (void)app_state_submit_command(APP_COMMAND_START);
    }
    else if (button_event == BUTTON_EVENT_STOP)
    {
      (void)app_state_submit_command(APP_COMMAND_STOP);
    }
    else if (button_event == BUTTON_EVENT_RETURN)
    {
      (void)app_state_submit_command(APP_COMMAND_RETURN);
    }
    else if (button_event == BUTTON_EVENT_PAGE)
    {
      (void)app_state_submit_command(APP_COMMAND_PAGE);
    }

    while (app_state_wait_command(&command, 0U))
    {
      app_tasks_apply_command(command);
    }

    app_state_set_pot_params(&effective);
    mode = app_state_get_mode();

    if ((mode == APP_MODE_STOP) || (mode == APP_MODE_FAULT) || (mode == APP_MODE_IDLE))
    {
      motor_control_enable(false);
    }

    app_platform_status_led_write((mode == APP_MODE_RUN) || (mode == APP_MODE_RETURN));
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_SUPERVISOR_PERIOD_MS));
  }
}

static void app_task_control(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  wheel_setpoint_t zero_setpoint = {0};

  (void)argument;
  for (;;)
  {
    wheel_state_t wheels = encoder_sample((float)APP_CONTROL_PERIOD_MS / 1000.0f);
    const app_mode_t mode = app_state_get_mode();

    app_state_set_wheel_state(&wheels);
    safety_periodic_check(&wheels);

    if ((mode == APP_MODE_RUN) || (mode == APP_MODE_RETURN))
    {
      motor_control_enable(true);
      motor_control_update((float)APP_CONTROL_PERIOD_MS / 1000.0f, &wheels);
    }
    else
    {
      zero_setpoint.timestamp_ms = HAL_GetTick();
      app_state_set_wheel_setpoint(&zero_setpoint);
      motor_control_safe_stop();
      motor_control_enable(false);
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
  }
}

static void app_task_perception(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();

  (void)argument;
  for (;;)
  {
    const app_snapshot_t snapshot = app_state_get_snapshot();
    const imu_state_t imu = imu_mpu6500_poll();
    pose2d_t pose;
    lidar_frame_t frame;

    lidar_process_dma();
    frame = lidar_get_latest();
    pose = odometry_update(&snapshot.wheels, &imu, (float)APP_PERCEPTION_PERIOD_MS / 1000.0f);

    app_state_set_lidar_frame(&frame);
    app_state_set_imu_state(&imu);
    app_state_set_pose(&pose);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_PERCEPTION_PERIOD_MS));
  }
}

static void app_task_ui(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_tx_ms = 0U;

  (void)argument;
  for (;;)
  {
    telemetry_bt_process_rx();
    ui_oled_render();

    if ((HAL_GetTick() - last_tx_ms) >= APP_TELEMETRY_PERIOD_MS)
    {
      telemetry_bt_send_periodic();
      last_tx_ms = HAL_GetTick();
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_UI_PERIOD_MS));
  }
}

bool app_tasks_bootstrap(void)
{
  bool ok = true;

  ok &= app_state_init();
  ok &= app_platform_init();
  ok &= motor_control_init();
  ok &= encoder_init();
  buttons_init();
  safety_init();
  odometry_init();

  if (!ok)
  {
    return false;
  }

  if (!app_platform_start_pwm())
  {
    safety_handle_fault(APP_FAULT_PWM_START);
  }
  if (!app_platform_start_encoders())
  {
    safety_handle_fault(APP_FAULT_ENCODER_START);
  }
  if (!app_platform_start_adc_dma())
  {
    safety_handle_fault(APP_FAULT_ADC_START);
  }
  if (!app_platform_start_lidar_dma())
  {
    safety_handle_fault(APP_FAULT_LIDAR_UART);
  }

  (void)lidar_init();

  if (!telemetry_bt_init())
  {
    app_state_set_fault(APP_FAULT_BT_UART);
  }
  if (!ui_oled_init())
  {
    app_state_set_fault(APP_FAULT_OLED_INIT);
  }
  if (!imu_mpu6500_init())
  {
    app_state_set_fault(APP_FAULT_IMU_INIT);
  }

  ok &= (xTaskCreate(app_task_supervisor, "task_supervisor", 384U, NULL, tskIDLE_PRIORITY + 3U, &g_supervisor_task) == pdPASS);
  ok &= (xTaskCreate(app_task_control, "task_control", 384U, NULL, tskIDLE_PRIORITY + 4U, &g_control_task) == pdPASS);
  ok &= (xTaskCreate(app_task_perception, "task_perception", 512U, NULL, tskIDLE_PRIORITY + 3U, &g_perception_task) == pdPASS);
  ok &= (xTaskCreate(app_task_ui, "task_ui", 512U, NULL, tskIDLE_PRIORITY + 2U, &g_ui_task) == pdPASS);

  return ok;
}
