#include "app_platform.h"

#include "app_config.h"
#include "main.h"
#include "semphr.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static uint16_t g_adc_buffer[APP_ADC_CHANNEL_COUNT];
static uint8_t g_lidar_dma_buffer[APP_LIDAR_DMA_BUFFER_SIZE];
static SemaphoreHandle_t g_i2c_mutex;

bool app_platform_init(void)
{
  if (g_i2c_mutex == NULL)
  {
    g_i2c_mutex = xSemaphoreCreateMutex();
  }
  return (g_i2c_mutex != NULL);
}

bool app_platform_start_pwm(void)
{
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    return false;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  return true;
}

bool app_platform_start_encoders(void)
{
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return false;
  }
  return true;
}

bool app_platform_start_adc_dma(void)
{
  return (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_buffer, APP_ADC_CHANNEL_COUNT) == HAL_OK);
}

bool app_platform_start_lidar_dma(void)
{
  if (HAL_UART_Receive_DMA(&huart1, g_lidar_dma_buffer, sizeof(g_lidar_dma_buffer)) != HAL_OK)
  {
    return false;
  }

  if (huart1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
  return true;
}

uint16_t *app_platform_get_adc_buffer(void)
{
  return g_adc_buffer;
}

uint8_t *app_platform_get_lidar_dma_buffer(void)
{
  return g_lidar_dma_buffer;
}

size_t app_platform_get_lidar_dma_buffer_size(void)
{
  return sizeof(g_lidar_dma_buffer);
}

TIM_HandleTypeDef *app_platform_motor_tim(void)
{
  return &htim3;
}

TIM_HandleTypeDef *app_platform_left_encoder_tim(void)
{
  return &htim2;
}

TIM_HandleTypeDef *app_platform_right_encoder_tim(void)
{
  return &htim4;
}

UART_HandleTypeDef *app_platform_lidar_uart(void)
{
  return &huart1;
}

UART_HandleTypeDef *app_platform_bt_uart(void)
{
  return &huart2;
}

I2C_HandleTypeDef *app_platform_i2c(void)
{
  return &hi2c1;
}

bool app_platform_i2c_acquire(TickType_t timeout_ticks)
{
  if (g_i2c_mutex == NULL)
  {
    return false;
  }
  return (xSemaphoreTake(g_i2c_mutex, timeout_ticks) == pdTRUE);
}

void app_platform_i2c_release(void)
{
  if (g_i2c_mutex != NULL)
  {
    (void)xSemaphoreGive(g_i2c_mutex);
  }
}

void app_platform_status_led_write(bool on)
{
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void app_platform_status_led_toggle(void)
{
  HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
}
