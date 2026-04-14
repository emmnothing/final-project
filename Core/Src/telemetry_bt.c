#include "telemetry_bt.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_state.h"
#include "app_platform.h"

typedef struct
{
  uint8_t rx_byte;
  uint8_t ring[APP_BT_RX_RING_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint8_t rx_seen;
  char line[APP_BT_LINE_BUFFER_SIZE];
  uint16_t line_len;
} telemetry_bt_ctx_t;

static telemetry_bt_ctx_t g_bt;

static void telemetry_bt_send_text(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  (void)HAL_UART_Transmit(app_platform_bt_uart(), (uint8_t *)text, (uint16_t)strlen(text), 50U);
}

static void telemetry_bt_send_ok(const char *suffix)
{
  char line[64];

  if (suffix == NULL)
  {
    suffix = "";
  }

  (void)snprintf(line, sizeof(line), "OK %s\r\n", suffix);
  telemetry_bt_send_text(line);
}

static void telemetry_bt_send_error(const char *suffix)
{
  char line[64];

  if (suffix == NULL)
  {
    suffix = "";
  }

  (void)snprintf(line, sizeof(line), "ERR %s\r\n", suffix);
  telemetry_bt_send_text(line);
}

static int telemetry_bt_match_command(const char *line, const char *command)
{
  return (strcmp(line, command) == 0);
}

static void telemetry_bt_parse_line(const char *line)
{
  float value = 0.0f;

  if ((line == NULL) || (line[0] == '\0'))
  {
    return;
  }

  if (telemetry_bt_match_command(line, "START"))
  {
    (void)app_state_submit_command(APP_COMMAND_START);
    telemetry_bt_send_ok("START");
    return;
  }
  if (telemetry_bt_match_command(line, "STOP"))
  {
    (void)app_state_submit_command(APP_COMMAND_STOP);
    telemetry_bt_send_ok("STOP");
    return;
  }
  if (telemetry_bt_match_command(line, "RETURN"))
  {
    (void)app_state_submit_command(APP_COMMAND_RETURN);
    telemetry_bt_send_ok("RETURN");
    return;
  }
  if (telemetry_bt_match_command(line, "PAGE"))
  {
    (void)app_state_submit_command(APP_COMMAND_PAGE);
    telemetry_bt_send_ok("PAGE");
    return;
  }
  if (telemetry_bt_match_command(line, "CLEARFAULT"))
  {
    (void)app_state_submit_command(APP_COMMAND_CLEAR_FAULT);
    telemetry_bt_send_ok("CLEARFAULT");
    return;
  }
  if (telemetry_bt_match_command(line, "RESETODOM"))
  {
    (void)app_state_submit_command(APP_COMMAND_RESET_ODOM);
    telemetry_bt_send_ok("RESETODOM");
    return;
  }
  if (telemetry_bt_match_command(line, "GET STATE") || telemetry_bt_match_command(line, "GET PARAMS"))
  {
    telemetry_bt_send_periodic();
    return;
  }
  if (telemetry_bt_match_command(line, "CLR OVERRIDE"))
  {
    app_state_clear_param_overrides();
    telemetry_bt_send_ok("CLR OVERRIDE");
    return;
  }
  if (sscanf(line, "SET SPEED_MAX=%f", &value) == 1)
  {
    if (app_state_set_param_override(APP_PARAM_SPEED_MAX, value))
    {
      telemetry_bt_send_ok("SET SPEED_MAX");
    }
    else
    {
      telemetry_bt_send_error("SET SPEED_MAX");
    }
    return;
  }
  if (sscanf(line, "SET TURN_MAX=%f", &value) == 1)
  {
    if (app_state_set_param_override(APP_PARAM_TURN_MAX, value))
    {
      telemetry_bt_send_ok("SET TURN_MAX");
    }
    else
    {
      telemetry_bt_send_error("SET TURN_MAX");
    }
    return;
  }
  if (sscanf(line, "SET SAFETY=%f", &value) == 1)
  {
    if (app_state_set_param_override(APP_PARAM_SAFETY_DISTANCE, value))
    {
      telemetry_bt_send_ok("SET SAFETY");
    }
    else
    {
      telemetry_bt_send_error("SET SAFETY");
    }
    return;
  }

  telemetry_bt_send_error("UNKNOWN");
}

bool telemetry_bt_init(void)
{
  memset(&g_bt, 0, sizeof(g_bt));
  if (HAL_UART_Receive_IT(app_platform_bt_uart(), &g_bt.rx_byte, 1U) != HAL_OK)
  {
    return false;
  }

  telemetry_bt_send_text("BT READY\r\n");
  return true;
}

void telemetry_bt_process_rx(void)
{
  if (g_bt.rx_seen != 0U)
  {
    g_bt.rx_seen = 0U;
    app_state_set_bt_link(true);
  }

  while (g_bt.tail != g_bt.head)
  {
    const uint8_t byte = g_bt.ring[g_bt.tail];
    g_bt.tail = (uint16_t)((g_bt.tail + 1U) % APP_BT_RX_RING_SIZE);

    if ((byte == '\r') || (byte == '\n'))
    {
      g_bt.line[g_bt.line_len] = '\0';
      telemetry_bt_parse_line(g_bt.line);
      g_bt.line_len = 0U;
      continue;
    }

    if (g_bt.line_len < (APP_BT_LINE_BUFFER_SIZE - 1U))
    {
      g_bt.line[g_bt.line_len++] = (char)byte;
    }
    else
    {
      g_bt.line_len = 0U;
      telemetry_bt_send_error("LINE TOO LONG");
    }
  }
}

void telemetry_bt_send_periodic(void)
{
  char line[196];
  const app_snapshot_t snapshot = app_state_get_snapshot();

  (void)snprintf(line,
                 sizeof(line),
                 "STATE mode=%s faults=0x%08lX page=%u x=%.3f y=%.3f yaw=%.3f wl=%.1f wr=%.1f spd=%.2f turn=%.2f safe=%.2f lidar=%lu/%lu imu=0x%02X gz=%.3f\r\n",
                 app_state_mode_string(snapshot.mode),
                 (unsigned long)snapshot.faults,
                 (unsigned int)snapshot.oled_page,
                 (double)snapshot.pose.x_m,
                 (double)snapshot.pose.y_m,
                 (double)snapshot.pose.yaw_rad,
                 (double)snapshot.wheels.left_ticks_per_s,
                 (double)snapshot.wheels.right_ticks_per_s,
                 (double)snapshot.params.speed_max_mps,
                 (double)snapshot.params.turn_max_radps,
                 (double)snapshot.params.safety_distance_m,
                 (unsigned long)snapshot.lidar.bytes_received,
                 (unsigned long)snapshot.lidar.frames_parsed,
                 (unsigned int)snapshot.imu.who_am_i,
                 (double)snapshot.imu.gyro_z_radps);
  telemetry_bt_send_text(line);
}

void telemetry_bt_on_rx_complete(UART_HandleTypeDef *huart)
{
  uint16_t next_head;

  if ((huart == NULL) || (huart->Instance != app_platform_bt_uart()->Instance))
  {
    return;
  }

  next_head = (uint16_t)((g_bt.head + 1U) % APP_BT_RX_RING_SIZE);
  if (next_head != g_bt.tail)
  {
    g_bt.ring[g_bt.head] = g_bt.rx_byte;
    g_bt.head = next_head;
  }

  g_bt.rx_seen = 1U;
  (void)HAL_UART_Receive_IT(app_platform_bt_uart(), &g_bt.rx_byte, 1U);
}
