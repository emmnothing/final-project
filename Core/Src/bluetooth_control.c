#include "bluetooth_control.h"

#include <stdio.h>
#include <string.h>

#include "lidar_pipeline.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern UART_HandleTypeDef huart6;

#define BLUETOOTH_RX_LINE_QUEUE_LENGTH 4U
#define BLUETOOTH_COMMAND_QUEUE_LENGTH 8U
#define BLUETOOTH_RETRY_RX_MS          1000U
#define BLUETOOTH_RX_ONLY_DEBUG        0U
#define BLUETOOTH_LIDAR_SYNC_0         0xA5U
#define BLUETOOTH_LIDAR_SYNC_1         0x5AU
#define BLUETOOTH_LIDAR_FRAME_TYPE     0xC1U
#define BLUETOOTH_LIDAR_POINTS_PER_FRAME 16U
#define BLUETOOTH_LIDAR_POINT_BYTES    6U
#define BLUETOOTH_LIDAR_HEADER_BYTES   10U
#define BLUETOOTH_LIDAR_FRAMES_PER_UPDATE 2U

typedef struct
{
  uint32_t tick_ms;
  char text[BLUETOOTH_LINE_MAX];
} BluetoothLine_t;

static StaticQueue_t s_rx_line_queue_struct;
static uint8_t s_rx_line_queue_storage[BLUETOOTH_RX_LINE_QUEUE_LENGTH * sizeof(BluetoothLine_t)];
static QueueHandle_t s_rx_line_queue;

static StaticQueue_t s_command_queue_struct;
static uint8_t s_command_queue_storage[BLUETOOTH_COMMAND_QUEUE_LENGTH * sizeof(BluetoothCommand_t)];
static QueueHandle_t s_command_queue;

static BluetoothControlState_t s_state;
static uint8_t s_rx_byte;
static char s_rx_line[BLUETOOTH_LINE_MAX];
static uint8_t s_rx_line_len;
static bool s_initialized;
static uint32_t s_last_rx_retry_tick_ms;
static uint16_t s_lidar_frame_sequence;

static bool BluetoothControl_StartReceive(void);
static bool BluetoothControl_SendBytes(const uint8_t *data, uint16_t length);
static void BluetoothControl_ProcessLine(const BluetoothLine_t *line);
static BluetoothCommandType_t BluetoothControl_ParseLine(const char *line);
static void BluetoothControl_NormalizeLine(const char *input, char *output, size_t output_size);
static void BluetoothControl_ApplyCommand(BluetoothCommandType_t command);
static void BluetoothControl_QueueCommand(BluetoothCommandType_t command, const char *text, uint32_t tick_ms);
static void BluetoothControl_SendAck(BluetoothCommandType_t command);
#if (BLUETOOTH_RX_ONLY_DEBUG == 0U)
static void BluetoothControl_SendLidarFrames(void);
static void BluetoothControl_SendLidarSummary(void);
static uint8_t BluetoothControl_FrameChecksum(const uint8_t *data, uint16_t length);
#endif
static bool BluetoothControl_IsLineBreak(uint8_t byte);
static bool BluetoothControl_IsPrintable(uint8_t byte);
static char BluetoothControl_ToUpper(char c);

bool BluetoothControl_Init(void)
{
  if (s_initialized)
  {
    return s_state.ready;
  }

  memset(&s_state, 0, sizeof(s_state));

  s_rx_line_queue = xQueueCreateStatic(
      BLUETOOTH_RX_LINE_QUEUE_LENGTH,
      sizeof(BluetoothLine_t),
      s_rx_line_queue_storage,
      &s_rx_line_queue_struct);
  configASSERT(s_rx_line_queue != NULL);

  s_command_queue = xQueueCreateStatic(
      BLUETOOTH_COMMAND_QUEUE_LENGTH,
      sizeof(BluetoothCommand_t),
      s_command_queue_storage,
      &s_command_queue_struct);
  configASSERT(s_command_queue != NULL);

  s_initialized = true;
  s_state.ready = BluetoothControl_StartReceive();
  return s_state.ready;
}

void BluetoothControl_Update(void)
{
  BluetoothLine_t line;
  uint32_t now = HAL_GetTick();

  if (!s_initialized)
  {
    (void)BluetoothControl_Init();
  }

  if (!s_state.ready && ((now - s_last_rx_retry_tick_ms) >= BLUETOOTH_RETRY_RX_MS))
  {
    s_last_rx_retry_tick_ms = now;
    s_state.ready = BluetoothControl_StartReceive();
  }

  while (xQueueReceive(s_rx_line_queue, &line, 0U) == pdPASS)
  {
    BluetoothControl_ProcessLine(&line);
  }

#if (BLUETOOTH_RX_ONLY_DEBUG == 0U)
  BluetoothControl_SendLidarFrames();
#endif
}

bool BluetoothControl_TakeCommand(BluetoothCommand_t *out_command)
{
  if ((out_command == NULL) || (s_command_queue == NULL))
  {
    return false;
  }

  return (xQueueReceive(s_command_queue, out_command, 0U) == pdPASS);
}

bool BluetoothControl_GetState(BluetoothControlState_t *out_state)
{
  if (out_state == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out_state = s_state;
  taskEXIT_CRITICAL();
  return s_initialized;
}

bool BluetoothControl_SendText(const char *text)
{
  size_t length;

  if (text == NULL)
  {
    return false;
  }

  length = strlen(text);
  if (length == 0U)
  {
    return true;
  }

  return BluetoothControl_SendBytes((const uint8_t *)text, (uint16_t)length);
}

static bool BluetoothControl_SendBytes(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return true;
  }

  if (HAL_UART_Transmit(&huart6, (uint8_t *)data, length, 50U) != HAL_OK)
  {
    s_state.ready = false;
    return false;
  }

  s_state.tx_count++;
  s_state.last_tx_tick_ms = HAL_GetTick();
  return true;
}

const char *BluetoothControl_CommandName(BluetoothCommandType_t command)
{
  switch (command)
  {
    case BLUETOOTH_CMD_START_MAPPING:   return "START_MAPPING";
    case BLUETOOTH_CMD_STOP_ALL:        return "STOP_ALL";
    case BLUETOOTH_CMD_SHOW_MAP_RESULT: return "SHOW_MAP";
    case BLUETOOTH_CMD_DEBUG_ON:        return "DEBUG_ON";
    case BLUETOOTH_CMD_DEBUG_OFF:       return "DEBUG_OFF";
    case BLUETOOTH_CMD_DRIVE_FORWARD:   return "DRIVE_FORWARD";
    case BLUETOOTH_CMD_TURN_LEFT:       return "TURN_LEFT";
    case BLUETOOTH_CMD_TURN_RIGHT:      return "TURN_RIGHT";
    case BLUETOOTH_CMD_DRIVE_STOP:      return "DRIVE_STOP";
    case BLUETOOTH_CMD_UNKNOWN:         return "UNKNOWN";
    case BLUETOOTH_CMD_NONE:
    default:                            return "NONE";
  }
}

void BluetoothControl_OnUartRxCpltFromIsr(UART_HandleTypeDef *huart)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  BluetoothLine_t completed_line;
  uint8_t byte;

  if ((huart == NULL) || (huart->Instance != USART6) || (s_rx_line_queue == NULL))
  {
    return;
  }

  byte = s_rx_byte;
  s_state.rx_bytes++;
  s_state.last_rx_tick_ms = HAL_GetTick();

  if (BluetoothControl_IsLineBreak(byte))
  {
    if (s_rx_line_len > 0U)
    {
      s_rx_line[s_rx_line_len] = '\0';
      completed_line.tick_ms = HAL_GetTick();
      memcpy(completed_line.text, s_rx_line, sizeof(completed_line.text));

      if (xQueueSendFromISR(s_rx_line_queue, &completed_line, &higher_priority_task_woken) != pdPASS)
      {
        s_state.command_drops++;
      }
      s_rx_line_len = 0U;
    }
  }
  else if (BluetoothControl_IsPrintable(byte))
  {
    if (s_rx_line_len < (BLUETOOTH_LINE_MAX - 1U))
    {
      s_rx_line[s_rx_line_len++] = (char)byte;
    }
    else
    {
      s_rx_line_len = 0U;
      s_state.rx_overflows++;
    }
  }

  s_state.ready = BluetoothControl_StartReceive();
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void BluetoothControl_OnUartErrorFromIsr(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART6))
  {
    return;
  }

  s_state.uart_errors++;
  s_state.ready = BluetoothControl_StartReceive();
}

static bool BluetoothControl_StartReceive(void)
{
  return (HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1U) == HAL_OK);
}

static void BluetoothControl_ProcessLine(const BluetoothLine_t *line)
{
  char normalized[BLUETOOTH_LINE_MAX];
  BluetoothCommandType_t command;

  BluetoothControl_NormalizeLine(line->text, normalized, sizeof(normalized));
  command = BluetoothControl_ParseLine(normalized);

  strncpy(s_state.last_line, normalized, sizeof(s_state.last_line) - 1U);
  s_state.last_line[sizeof(s_state.last_line) - 1U] = '\0';
  s_state.rx_lines++;

#if (BLUETOOTH_RX_ONLY_DEBUG != 0U)
  s_state.last_command = BLUETOOTH_CMD_NONE;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  return;
#endif

  if (command == BLUETOOTH_CMD_UNKNOWN)
  {
    s_state.parse_errors++;
    (void)BluetoothControl_SendText("ERR CMD\r\n");
    return;
  }

  BluetoothControl_ApplyCommand(command);
  BluetoothControl_QueueCommand(command, normalized, line->tick_ms);
  BluetoothControl_SendAck(command);

  if (command == BLUETOOTH_CMD_SHOW_MAP_RESULT)
  {
    BluetoothControl_SendLidarSummary();
  }
}

static BluetoothCommandType_t BluetoothControl_ParseLine(const char *line)
{
  if ((strcmp(line, "START") == 0) ||
      (strcmp(line, "MAP START") == 0) ||
      (strcmp(line, "START MAP") == 0))
  {
    return BLUETOOTH_CMD_START_MAPPING;
  }

  if ((strcmp(line, "STOP") == 0) ||
      (strcmp(line, "HALT") == 0) ||
      (strcmp(line, "MAP STOP") == 0))
  {
    return BLUETOOTH_CMD_STOP_ALL;
  }

  if ((strcmp(line, "SHOW") == 0) ||
      (strcmp(line, "RESULT") == 0) ||
      (strcmp(line, "MAP SHOW") == 0) ||
      (strcmp(line, "SHOW MAP") == 0))
  {
    return BLUETOOTH_CMD_SHOW_MAP_RESULT;
  }

  if ((strcmp(line, "DEBUG ON") == 0) ||
      (strcmp(line, "DBG ON") == 0))
  {
    return BLUETOOTH_CMD_DEBUG_ON;
  }

  if ((strcmp(line, "DEBUG OFF") == 0) ||
      (strcmp(line, "DBG OFF") == 0))
  {
    return BLUETOOTH_CMD_DEBUG_OFF;
  }

  if ((strcmp(line, "FWD") == 0) ||
      (strcmp(line, "FORWARD") == 0) ||
      (strcmp(line, "GO") == 0))
  {
    return BLUETOOTH_CMD_DRIVE_FORWARD;
  }

  if ((strcmp(line, "LEFT") == 0) ||
      (strcmp(line, "TURN LEFT") == 0))
  {
    return BLUETOOTH_CMD_TURN_LEFT;
  }

  if ((strcmp(line, "RIGHT") == 0) ||
      (strcmp(line, "TURN RIGHT") == 0))
  {
    return BLUETOOTH_CMD_TURN_RIGHT;
  }

  if ((strcmp(line, "DRIVE STOP") == 0) ||
      (strcmp(line, "MOTOR STOP") == 0) ||
      (strcmp(line, "BRAKE") == 0))
  {
    return BLUETOOTH_CMD_DRIVE_STOP;
  }

  return BLUETOOTH_CMD_UNKNOWN;
}

static void BluetoothControl_NormalizeLine(const char *input, char *output, size_t output_size)
{
  size_t in_index;
  size_t out_index = 0U;
  bool previous_space = true;

  if (output_size == 0U)
  {
    return;
  }

  for (in_index = 0U; (input[in_index] != '\0') && (out_index < (output_size - 1U)); ++in_index)
  {
    char c = input[in_index];

    if ((c == '\t') || (c == '-') || (c == '_'))
    {
      c = ' ';
    }

    if (c == ' ')
    {
      if (!previous_space)
      {
        output[out_index++] = ' ';
      }
      previous_space = true;
      continue;
    }

    output[out_index++] = BluetoothControl_ToUpper(c);
    previous_space = false;
  }

  if ((out_index > 0U) && (output[out_index - 1U] == ' '))
  {
    out_index--;
  }
  output[out_index] = '\0';
}

static void BluetoothControl_ApplyCommand(BluetoothCommandType_t command)
{
  s_state.last_command = command;

  if (command == BLUETOOTH_CMD_START_MAPPING)
  {
    s_state.mapping_active = true;
  }
  else if (command == BLUETOOTH_CMD_STOP_ALL)
  {
    s_state.mapping_active = false;
  }
  else if (command == BLUETOOTH_CMD_DEBUG_ON)
  {
    s_state.debug_enabled = true;
  }
  else if (command == BLUETOOTH_CMD_DEBUG_OFF)
  {
    s_state.debug_enabled = false;
  }
}

static void BluetoothControl_QueueCommand(BluetoothCommandType_t command, const char *text, uint32_t tick_ms)
{
  BluetoothCommand_t item;

  if (s_command_queue == NULL)
  {
    return;
  }

  memset(&item, 0, sizeof(item));
  item.type = command;
  item.tick_count = tick_ms;
  strncpy(item.text, text, sizeof(item.text) - 1U);

  if (xQueueSend(s_command_queue, &item, 0U) != pdPASS)
  {
    s_state.command_drops++;
  }
}

static void BluetoothControl_SendAck(BluetoothCommandType_t command)
{
  char response[48];

  (void)snprintf(response, sizeof(response), "ACK %s\r\n", BluetoothControl_CommandName(command));
  (void)BluetoothControl_SendText(response);
}

#if (BLUETOOTH_RX_ONLY_DEBUG == 0U)
static void BluetoothControl_SendLidarFrames(void)
{
  uint8_t frame[BLUETOOTH_LIDAR_HEADER_BYTES +
                (BLUETOOTH_LIDAR_POINTS_PER_FRAME * BLUETOOTH_LIDAR_POINT_BYTES) +
                1U];
  uint8_t frames_sent;

  for (frames_sent = 0U; frames_sent < BLUETOOTH_LIDAR_FRAMES_PER_UPDATE; ++frames_sent)
  {
    uint8_t count = 0U;
    uint8_t index;
    uint16_t sequence = s_lidar_frame_sequence++;
    uint32_t drops = LidarPipeline_GetPointQueueDrops();
    LidarPoint_t point;

    frame[0] = BLUETOOTH_LIDAR_SYNC_0;
    frame[1] = BLUETOOTH_LIDAR_SYNC_1;
    frame[2] = BLUETOOTH_LIDAR_FRAME_TYPE;
    frame[3] = 0U;
    frame[4] = (uint8_t)(sequence & 0xFFU);
    frame[5] = (uint8_t)((sequence >> 8U) & 0xFFU);
    frame[6] = (uint8_t)(drops & 0xFFU);
    frame[7] = (uint8_t)((drops >> 8U) & 0xFFU);
    frame[8] = (uint8_t)((drops >> 16U) & 0xFFU);
    frame[9] = (uint8_t)((drops >> 24U) & 0xFFU);

    while ((count < BLUETOOTH_LIDAR_POINTS_PER_FRAME) && LidarPipeline_TakePoint(&point))
    {
      index = (uint8_t)(BLUETOOTH_LIDAR_HEADER_BYTES + (count * BLUETOOTH_LIDAR_POINT_BYTES));
      frame[index] = (uint8_t)(point.angle_cdeg & 0xFFU);
      frame[index + 1U] = (uint8_t)((point.angle_cdeg >> 8U) & 0xFFU);
      frame[index + 2U] = (uint8_t)(point.distance_mm & 0xFFU);
      frame[index + 3U] = (uint8_t)((point.distance_mm >> 8U) & 0xFFU);
      frame[index + 4U] = point.quality;
      frame[index + 5U] = point.flags;
      count++;
    }

    if (count == 0U)
    {
      return;
    }

    frame[3] = count;
    index = (uint8_t)(BLUETOOTH_LIDAR_HEADER_BYTES + (count * BLUETOOTH_LIDAR_POINT_BYTES));
    frame[index] = BluetoothControl_FrameChecksum(frame, index);

    if (!BluetoothControl_SendBytes(frame, (uint16_t)(index + 1U)))
    {
      return;
    }
  }
}

static void BluetoothControl_SendLidarSummary(void)
{
  LidarParseResult_t lidar;
  char line[128];

  if (!LidarPipeline_GetLatestResult(&lidar))
  {
    (void)BluetoothControl_SendText("LIDAR wait\r\n");
    return;
  }

  (void)snprintf(
      line,
      sizeof(line),
      "LIDAR bytes=%lu valid=%u invalid=%u dist=%u min=%u angle=%u.%02u qual=%u err=%lu\r\n",
      (unsigned long)lidar.total_bytes,
      (unsigned int)lidar.valid_nodes,
      (unsigned int)lidar.invalid_nodes,
      (unsigned int)lidar.last_distance_mm,
      (unsigned int)lidar.min_distance_mm,
      (unsigned int)(lidar.last_angle_cdeg / 100U),
      (unsigned int)(lidar.last_angle_cdeg % 100U),
      (unsigned int)lidar.last_quality,
      (unsigned long)(lidar.uart_error_count + lidar.dma_queue_drops + lidar.result_queue_drops + lidar.point_queue_drops));
  (void)BluetoothControl_SendText(line);
}

static uint8_t BluetoothControl_FrameChecksum(const uint8_t *data, uint16_t length)
{
  uint8_t checksum = 0U;
  uint16_t index;

  if (data == NULL)
  {
    return 0U;
  }

  for (index = 2U; index < length; ++index)
  {
    checksum ^= data[index];
  }

  return checksum;
}
#endif

static bool BluetoothControl_IsLineBreak(uint8_t byte)
{
  return ((byte == '\r') || (byte == '\n'));
}

static bool BluetoothControl_IsPrintable(uint8_t byte)
{
  return ((byte >= 0x20U) && (byte <= 0x7EU));
}

static char BluetoothControl_ToUpper(char c)
{
  if ((c >= 'a') && (c <= 'z'))
  {
    return (char)(c - ('a' - 'A'));
  }

  return c;
}
