/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bluetooth_control.h"
#include "lidar_pipeline.h"
#include "motor_control.h"
#include "mpu6500.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_I2C_ADDR               (0x3CU << 1)
#define OLED_WIDTH                  128U
#define OLED_HEIGHT                 64U
#define OLED_PAGE_COUNT             (OLED_HEIGHT / 8U)
#define OLED_FB_SIZE                (OLED_WIDTH * OLED_PAGE_COUNT)

#define ADC_CHANNEL_COUNT           3U
#define BUTTON_DEBOUNCE_MS          30U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
osThreadId senseTaskHandle;
osThreadId uiTaskHandle;
osThreadId btTaskHandle;
/* USER CODE BEGIN PV */
typedef enum
{
  TEST_PAGE_LIDAR = 0,
  TEST_PAGE_MPU,
  TEST_PAGE_ADC,
  TEST_PAGE_ENCODER,
  TEST_PAGE_BT,
  TEST_PAGE_COUNT
} test_page_t;

typedef struct
{
  uint16_t last_left_counter;
  uint16_t last_right_counter;
  int16_t left_delta;
  int16_t right_delta;
  int32_t left_total;
  int32_t right_total;
} encoder_test_state_t;

static uint16_t adc_buf[ADC_CHANNEL_COUNT];
static uint8_t oled_fb[OLED_FB_SIZE];
static uint8_t oled_page_tx[OLED_WIDTH + 1U];

static volatile bool app_ready = false;
static bool oled_ready = false;
static test_page_t current_page = TEST_PAGE_LIDAR;
static uint8_t button_stable_mask = 0U;
static uint8_t button_prev_nonzero = 0U;
static uint8_t button_sample_mask = 0U;
static uint32_t button_change_tick_ms = 0U;
static uint32_t last_lidar_result_tick_ms = 0U;
static uint32_t last_oled_tick_ms = 0U;
static uint32_t last_sensor_tick_ms = 0U;

static bool lidar_result_valid = false;
static LidarParseResult_t lidar_result = {0};
static Mpu6500State_t mpu_state = {0};
static encoder_test_state_t encoder_test = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);
void StartSenseTask(void const * argument);
void StartUiTask(void const * argument);
void StartBtTask(void const * argument);

/* USER CODE BEGIN PFP */
static void TestApp_InitPeripherals(void);
static void TestApp_PollButtons(void);
static void TestApp_UpdateSensors(void);
static void TestApp_UpdateDisplay(void);

static bool OLED_InitMinimal(void);
static bool OLED_WriteCommand(uint8_t command);
static bool OLED_WriteCommands(const uint8_t *commands, uint16_t length);
static void OLED_Clear(void);
static void OLED_Update(void);
static void OLED_DrawChar6x8(uint8_t x, uint8_t page, char c);
static void OLED_DrawString6x8(uint8_t x, uint8_t page, const char *text);

static int16_t Encoder_ComputeDelta(uint16_t current, uint16_t previous);
static uint8_t Buttons_ReadMask(void);
static void RenderPage_LiDAR(void);
static void RenderPage_MPU(void);
static void RenderPage_ADC(void);
static void RenderPage_Encoder(void);
static void RenderPage_Bluetooth(void);
static void DrawStatusLine(uint8_t page, const char *label, int32_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint8_t font_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t font_dash[5]  = {0x08, 0x08, 0x08, 0x08, 0x08};
static const uint8_t font_dot[5]   = {0x00, 0x60, 0x60, 0x00, 0x00};
static const uint8_t font_colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
static const uint8_t font_slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
static const uint8_t font_equal[5] = {0x14, 0x14, 0x14, 0x14, 0x14};
static const uint8_t font_0[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
static const uint8_t font_1[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
static const uint8_t font_2[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
static const uint8_t font_3[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
static const uint8_t font_4[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
static const uint8_t font_5[5] = {0x27, 0x45, 0x45, 0x45, 0x39};
static const uint8_t font_6[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30};
static const uint8_t font_7[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
static const uint8_t font_8[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
static const uint8_t font_9[5] = {0x06, 0x49, 0x49, 0x29, 0x1E};
static const uint8_t font_A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
static const uint8_t font_B[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
static const uint8_t font_C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
static const uint8_t font_D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
static const uint8_t font_E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
static const uint8_t font_F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
static const uint8_t font_G[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A};
static const uint8_t font_H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
static const uint8_t font_I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
static const uint8_t font_J[5] = {0x20, 0x40, 0x41, 0x3F, 0x01};
static const uint8_t font_K[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
static const uint8_t font_L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
static const uint8_t font_M[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
static const uint8_t font_N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
static const uint8_t font_O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
static const uint8_t font_P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
static const uint8_t font_Q[5] = {0x3E, 0x41, 0x51, 0x21, 0x5E};
static const uint8_t font_R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
static const uint8_t font_S[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
static const uint8_t font_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
static const uint8_t font_U[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
static const uint8_t font_V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
static const uint8_t font_W[5] = {0x7F, 0x20, 0x18, 0x20, 0x7F};
static const uint8_t font_X[5] = {0x63, 0x14, 0x08, 0x14, 0x63};
static const uint8_t font_Y[5] = {0x03, 0x04, 0x78, 0x04, 0x03};
static const uint8_t font_Z[5] = {0x61, 0x51, 0x49, 0x45, 0x43};

static const uint8_t *OLED_GetGlyph(char c)
{
  switch (c)
  {
    case ' ': return font_space;
    case '-': return font_dash;
    case '.': return font_dot;
    case ':': return font_colon;
    case '/': return font_slash;
    case '=': return font_equal;
    case '0': return font_0;
    case '1': return font_1;
    case '2': return font_2;
    case '3': return font_3;
    case '4': return font_4;
    case '5': return font_5;
    case '6': return font_6;
    case '7': return font_7;
    case '8': return font_8;
    case '9': return font_9;
    case 'A': return font_A;
    case 'B': return font_B;
    case 'C': return font_C;
    case 'D': return font_D;
    case 'E': return font_E;
    case 'F': return font_F;
    case 'G': return font_G;
    case 'H': return font_H;
    case 'I': return font_I;
    case 'J': return font_J;
    case 'K': return font_K;
    case 'L': return font_L;
    case 'M': return font_M;
    case 'N': return font_N;
    case 'O': return font_O;
    case 'P': return font_P;
    case 'Q': return font_Q;
    case 'R': return font_R;
    case 'S': return font_S;
    case 'T': return font_T;
    case 'U': return font_U;
    case 'V': return font_V;
    case 'W': return font_W;
    case 'X': return font_X;
    case 'Y': return font_Y;
    case 'Z': return font_Z;
    default:  return font_space;
  }
}

static bool OLED_WriteCommand(uint8_t command)
{
  uint8_t payload[2] = {0x00U, command};
  return (HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, payload, sizeof(payload), 100U) == HAL_OK);
}

static bool OLED_WriteCommands(const uint8_t *commands, uint16_t length)
{
  uint16_t i;

  for (i = 0U; i < length; ++i)
  {
    if (!OLED_WriteCommand(commands[i]))
    {
      return false;
    }
  }

  return true;
}

static bool OLED_InitMinimal(void)
{
  static const uint8_t init_cmds[] =
  {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0x8DU, 0x14U, 0x20U, 0x00U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
    0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
    0xAFU
  };

  if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_I2C_ADDR, 3U, 50U) != HAL_OK)
  {
    return false;
  }

  if (!OLED_WriteCommands(init_cmds, sizeof(init_cmds)))
  {
    return false;
  }

  OLED_Clear();
  OLED_Update();
  return true;
}

static void OLED_Clear(void)
{
  memset(oled_fb, 0, sizeof(oled_fb));
}

static void OLED_DrawChar6x8(uint8_t x, uint8_t page, char c)
{
  const uint8_t *glyph;
  uint16_t offset;
  uint8_t col;

  if ((x > (OLED_WIDTH - 6U)) || (page >= OLED_PAGE_COUNT))
  {
    return;
  }

  glyph = OLED_GetGlyph(c);
  offset = ((uint16_t)page * OLED_WIDTH) + x;

  for (col = 0U; col < 5U; ++col)
  {
    oled_fb[offset + col] = glyph[col];
  }
  oled_fb[offset + 5U] = 0x00U;
}

static void OLED_DrawString6x8(uint8_t x, uint8_t page, const char *text)
{
  while ((*text != '\0') && (x <= (OLED_WIDTH - 6U)))
  {
    OLED_DrawChar6x8(x, page, *text);
    x = (uint8_t)(x + 6U);
    ++text;
  }
}

static void OLED_Update(void)
{
  uint8_t page;
  uint8_t commands[3];

  if (!oled_ready)
  {
    return;
  }

  oled_page_tx[0] = 0x40U;

  for (page = 0U; page < OLED_PAGE_COUNT; ++page)
  {
    commands[0] = (uint8_t)(0xB0U + page);
    commands[1] = 0x00U;
    commands[2] = 0x10U;

    if (!OLED_WriteCommands(commands, sizeof(commands)))
    {
      oled_ready = false;
      return;
    }

    memcpy(&oled_page_tx[1], &oled_fb[page * OLED_WIDTH], OLED_WIDTH);
    if (HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, oled_page_tx, sizeof(oled_page_tx), 100U) != HAL_OK)
    {
      oled_ready = false;
      return;
    }
  }
}

static int16_t Encoder_ComputeDelta(uint16_t current, uint16_t previous)
{
  int32_t delta = (int32_t)current - (int32_t)previous;

  if (delta > 32767L)
  {
    delta -= 65536L;
  }
  else if (delta < -32768L)
  {
    delta += 65536L;
  }

  return (int16_t)delta;
}

static uint8_t Buttons_ReadMask(void)
{
  uint8_t mask = 0U;

  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) { mask |= 0x01U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) { mask |= 0x02U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) { mask |= 0x04U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET) { mask |= 0x08U; }

  return mask;
}

static void TestApp_PollButtons(void)
{
  uint8_t raw = Buttons_ReadMask();
  uint8_t page_button_pressed;
  uint32_t now = HAL_GetTick();

  if (raw != button_sample_mask)
  {
    button_sample_mask = raw;
    button_change_tick_ms = now;
  }

  if ((now - button_change_tick_ms) < BUTTON_DEBOUNCE_MS)
  {
    return;
  }

  if (button_stable_mask != button_sample_mask)
  {
    button_stable_mask = button_sample_mask;
    page_button_pressed = (uint8_t)(button_stable_mask & 0x08U);
    if ((page_button_pressed != 0U) && (button_prev_nonzero == 0U))
    {
      current_page = (test_page_t)((current_page + 1U) % TEST_PAGE_COUNT);
    }
    button_prev_nonzero = page_button_pressed;
  }
}

static void DrawStatusLine(uint8_t page, const char *label, int32_t value)
{
  char line[22];
  (void)snprintf(line, sizeof(line), "%-7s:%ld", label, (long)value);
  OLED_DrawString6x8(0U, page, line);
}

static void RenderPage_LiDAR(void)
{
  uint32_t now = HAL_GetTick();
  bool stream_alive = false;

  if (lidar_result_valid && ((now - last_lidar_result_tick_ms) < 1000U) && (lidar_result.valid_nodes > 0U))
  {
    stream_alive = true;
  }

  OLED_DrawString6x8(0U, 0U, "LIDAR TEST");
  OLED_DrawString6x8(0U, 1U, (lidar_result_valid && (lidar_result.descriptor_seen != 0U)) ? "DESC   : YES" : "DESC   : NO");
  OLED_DrawString6x8(0U, 2U, stream_alive ? "STREAM : OK" : "STREAM : NO");
  DrawStatusLine(3U, "BYTES", (int32_t)lidar_result.total_bytes);
  DrawStatusLine(4U, "VALID", (int32_t)lidar_result.valid_nodes);
  DrawStatusLine(5U, "INVAL", (int32_t)lidar_result.invalid_nodes);
  DrawStatusLine(6U, "DIST", (int32_t)lidar_result.last_distance_mm);
  DrawStatusLine(7U, "QUAL", (int32_t)lidar_result.last_quality);
}

static void RenderPage_MPU(void)
{
  char line[22];

  OLED_DrawString6x8(0U, 0U, "MPU6500 TEST");
  OLED_DrawString6x8(0U, 1U, mpu_state.ready ? "READY  : YES" : "READY  : NO");
  (void)snprintf(line, sizeof(line), "WHOAMI : %02X", mpu_state.who_am_i);
  OLED_DrawString6x8(0U, 2U, line);
  DrawStatusLine(3U, "GYROZ", (int32_t)mpu_state.gyro_z_raw);
  DrawStatusLine(4U, "DPSX100", mpu_state.gyro_z_dps_x100);
  OLED_DrawString6x8(0U, 6U, "BTN ANY TO PAGE");
  OLED_DrawString6x8(0U, 7U, "BT RX PAGE");
}

static void RenderPage_ADC(void)
{
  OLED_DrawString6x8(0U, 0U, "ADC TEST");
  DrawStatusLine(1U, "P1RAW", (int32_t)adc_buf[0]);
  DrawStatusLine(2U, "P2RAW", (int32_t)adc_buf[1]);
  DrawStatusLine(3U, "P3RAW", (int32_t)adc_buf[2]);
  DrawStatusLine(4U, "P1LIM", ((int32_t)adc_buf[0] * 100L) / 4095L);
  DrawStatusLine(5U, "P2LIM", ((int32_t)adc_buf[1] * 100L) / 4095L);
  DrawStatusLine(6U, "P3LIM", ((int32_t)adc_buf[2] * 100L) / 4095L);
  OLED_DrawString6x8(0U, 7U, "BTN ANY TO PAGE");
}

static void RenderPage_Encoder(void)
{
  MotorControlState_t motor_state = {0};

  (void)MotorControl_GetState(&motor_state);

  if (motor_state.mode == 1U)
  {
    OLED_DrawString6x8(0U, 0U, "CAL PWM ON");
  }
  else
  {
    OLED_DrawString6x8(0U, 0U, "CAL PWM OFF");
  }
  DrawStatusLine(1U, "LDELTA", (int32_t)motor_state.left_delta);
  DrawStatusLine(2U, "RDELTA", (int32_t)motor_state.right_delta);
  DrawStatusLine(3U, "LCNT", (int32_t)motor_state.left_counter);
  DrawStatusLine(4U, "RCNT", (int32_t)motor_state.right_counter);
  DrawStatusLine(5U, "LRAW", (int32_t)motor_state.left_encoder_raw);
  DrawStatusLine(6U, "RRAW", (int32_t)motor_state.right_encoder_raw);
  OLED_DrawString6x8(0U, 7U, "PC0 TOGGLE CAL");
}

static void RenderPage_Bluetooth(void)
{
  BluetoothControlState_t bt_state = {0};
  char line[22];

  (void)BluetoothControl_GetState(&bt_state);

  OLED_DrawString6x8(0U, 0U, "BT RX TEST");
  OLED_DrawString6x8(0U, 1U, bt_state.ready ? "READY  : YES" : "READY  : NO");
  DrawStatusLine(2U, "BYTES", (int32_t)bt_state.rx_bytes);
  DrawStatusLine(3U, "LINES", (int32_t)bt_state.rx_lines);
  DrawStatusLine(4U, "OVFL", (int32_t)bt_state.rx_overflows);
  DrawStatusLine(5U, "UARTERR", (int32_t)bt_state.uart_errors);
  (void)snprintf(line, sizeof(line), "LAST:%.16s", bt_state.last_line);
  OLED_DrawString6x8(0U, 6U, line);
  OLED_DrawString6x8(0U, 7U, "SEND TEXT PLUS CR");
}

static void TestApp_UpdateDisplay(void)
{
  uint32_t now = HAL_GetTick();

  if (!oled_ready || ((now - last_oled_tick_ms) < 150U))
  {
    return;
  }

  last_oled_tick_ms = now;
  OLED_Clear();

  switch (current_page)
  {
    case TEST_PAGE_LIDAR:
      RenderPage_LiDAR();
      break;
    case TEST_PAGE_MPU:
      RenderPage_MPU();
      break;
    case TEST_PAGE_ADC:
      RenderPage_ADC();
      break;
    case TEST_PAGE_ENCODER:
      RenderPage_Encoder();
      break;
    case TEST_PAGE_BT:
    default:
      RenderPage_Bluetooth();
      break;
  }

  OLED_Update();
}

static void TestApp_UpdateSensors(void)
{
  uint16_t left_now;
  uint16_t right_now;
  uint32_t now = HAL_GetTick();

  if ((now - last_sensor_tick_ms) < 20U)
  {
    return;
  }

  last_sensor_tick_ms = now;

  if (LidarPipeline_GetLatestResult(&lidar_result))
  {
    lidar_result_valid = true;
    last_lidar_result_tick_ms = now;
  }

  Mpu6500_Update();
  (void)Mpu6500_GetState(&mpu_state);

  left_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  right_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);

  encoder_test.left_delta = Encoder_ComputeDelta(left_now, encoder_test.last_left_counter);
  encoder_test.right_delta = Encoder_ComputeDelta(right_now, encoder_test.last_right_counter);
  encoder_test.left_total += encoder_test.left_delta;
  encoder_test.right_total += encoder_test.right_delta;
  encoder_test.last_left_counter = left_now;
  encoder_test.last_right_counter = right_now;
}

static void TestApp_InitPeripherals(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_CHANNEL_COUNT);
  (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  if (!MotorControl_Init())
  {
    Error_Handler();
  }

  encoder_test.last_left_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  encoder_test.last_right_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);

  oled_ready = OLED_InitMinimal();
  (void)Mpu6500_Init();
  (void)Mpu6500_GetState(&mpu_state);
  (void)BluetoothControl_Init();
  if (!LidarPipeline_Init())
  {
    Error_Handler();
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  TestApp_InitPeripherals();
  app_ready = true;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(senseTask, StartSenseTask, osPriorityAboveNormal, 0, 512);
  senseTaskHandle = osThreadCreate(osThread(senseTask), NULL);

  osThreadDef(uiTask, StartUiTask, osPriorityBelowNormal, 0, 512);
  uiTaskHandle = osThreadCreate(osThread(uiTask), NULL);

  osThreadDef(btTask, StartBtTask, osPriorityLow, 0, 256);
  btTaskHandle = osThreadCreate(osThread(btTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;

  vTaskDelete(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSenseTask */
/**
  * @brief  Function implementing the senseTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSenseTask */
void StartSenseTask(void const * argument)
{
  /* USER CODE BEGIN StartSenseTask */
  for(;;)
  {
    if (app_ready)
    {
      MotorControl_UpdateButtons();
      TestApp_UpdateSensors();
    }
    osDelay(5);
  }
  /* USER CODE END StartSenseTask */
}

/* USER CODE BEGIN Header_StartUiTask */
/**
  * @brief  Function implementing the uiTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUiTask */
void StartUiTask(void const * argument)
{
  /* USER CODE BEGIN StartUiTask */
  for(;;)
  {
    if (app_ready)
    {
      TestApp_PollButtons();
      TestApp_UpdateDisplay();
    }
    osDelay(20);
  }
  /* USER CODE END StartUiTask */
}

/* USER CODE BEGIN Header_StartBtTask */
/**
  * @brief  Function implementing the btTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBtTask */
void StartBtTask(void const * argument)
{
  /* USER CODE BEGIN StartBtTask */
  for(;;)
  {
    if (app_ready)
    {
      BluetoothControl_Update();
    }
    osDelay(20);
  }
  /* USER CODE END StartBtTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
