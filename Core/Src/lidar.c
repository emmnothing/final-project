#include "lidar.h"

#include <string.h>

#include "app_config.h"
#include "app_platform.h"

#define RPLIDAR_CMD_SYNC_BYTE      0xA5U
#define RPLIDAR_CMD_STOP           0x25U
#define RPLIDAR_CMD_SCAN           0x20U
#define RPLIDAR_DESC_SYNC_BYTE2    0x5AU
#define RPLIDAR_STANDARD_NODE_SIZE 5U

typedef struct
{
  size_t dma_last_pos;
  uint8_t descriptor_window[7];
  uint8_t node_buffer[RPLIDAR_STANDARD_NODE_SIZE];
  uint8_t node_index;
  uint16_t current_scan_samples;
  uint32_t last_byte_ms;
  lidar_frame_t frame;
} lidar_ctx_t;

static lidar_ctx_t g_lidar;

static void lidar_send_command(uint8_t command)
{
  uint8_t packet[2] = {RPLIDAR_CMD_SYNC_BYTE, command};
  (void)HAL_UART_Transmit(app_platform_lidar_uart(), packet, sizeof(packet), 20U);
}

static void lidar_shift_descriptor(uint8_t byte)
{
  memmove(&g_lidar.descriptor_window[0], &g_lidar.descriptor_window[1], sizeof(g_lidar.descriptor_window) - 1U);
  g_lidar.descriptor_window[sizeof(g_lidar.descriptor_window) - 1U] = byte;

  if ((g_lidar.descriptor_window[0] == RPLIDAR_CMD_SYNC_BYTE) &&
      (g_lidar.descriptor_window[1] == RPLIDAR_DESC_SYNC_BYTE2))
  {
    g_lidar.frame.parser_locked = true;
  }
}

static void lidar_try_parse_standard_node(void)
{
  const uint8_t *node = g_lidar.node_buffer;
  const uint8_t start_flag = node[0] & 0x01U;
  const uint8_t inverse_flag = (node[0] >> 1) & 0x01U;
  const bool sync_ok = (start_flag != inverse_flag);
  const bool check_ok = ((node[1] & 0x01U) != 0U);
  const uint16_t distance_q2 = (uint16_t)node[3] | ((uint16_t)node[4] << 8);

  if (!sync_ok || !check_ok || (distance_q2 == 0U))
  {
    return;
  }

  if (start_flag != 0U)
  {
    if (g_lidar.current_scan_samples > 0U)
    {
      g_lidar.frame.last_sample_count = g_lidar.current_scan_samples;
    }
    g_lidar.current_scan_samples = 0U;
  }

  ++g_lidar.current_scan_samples;
  ++g_lidar.frame.frames_parsed;
  g_lidar.frame.last_frame_ms = HAL_GetTick();
}

static void lidar_consume_byte(uint8_t byte)
{
  ++g_lidar.frame.bytes_received;
  g_lidar.last_byte_ms = HAL_GetTick();
  lidar_shift_descriptor(byte);

  g_lidar.node_buffer[g_lidar.node_index++] = byte;
  if (g_lidar.node_index >= RPLIDAR_STANDARD_NODE_SIZE)
  {
    lidar_try_parse_standard_node();
    g_lidar.node_index = 0U;
  }
}

bool lidar_init(void)
{
  memset(&g_lidar, 0, sizeof(g_lidar));
  lidar_send_command(RPLIDAR_CMD_STOP);
  HAL_Delay(2U);
  lidar_send_command(RPLIDAR_CMD_SCAN);
  return true;
}

void lidar_process_dma(void)
{
  UART_HandleTypeDef *uart = app_platform_lidar_uart();
  const size_t dma_size = app_platform_get_lidar_dma_buffer_size();
  uint8_t *dma_buffer = app_platform_get_lidar_dma_buffer();
  size_t dma_head;
  const uint32_t now = HAL_GetTick();

  if ((uart == NULL) || (uart->hdmarx == NULL))
  {
    g_lidar.frame.stream_active = false;
    return;
  }

  dma_head = dma_size - (size_t)__HAL_DMA_GET_COUNTER(uart->hdmarx);

  while (g_lidar.dma_last_pos != dma_head)
  {
    lidar_consume_byte(dma_buffer[g_lidar.dma_last_pos]);
    g_lidar.dma_last_pos = (g_lidar.dma_last_pos + 1U) % dma_size;
  }

  g_lidar.frame.stream_active = ((now - g_lidar.last_byte_ms) <= APP_LIDAR_STREAM_TIMEOUT_MS);
}

lidar_frame_t lidar_get_latest(void)
{
  return g_lidar.frame;
}

bool lidar_is_ready(void)
{
  return g_lidar.frame.stream_active;
}
