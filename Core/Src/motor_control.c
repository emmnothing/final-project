#include "motor_control.h"

#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

#define MOTOR_PWM_TIMER_MAX      999U
#define MOTOR_CAL_DUTY_PERMILLE  600U
#define MOTOR_PWM_CH1_MASK       0x01U
#define MOTOR_PWM_CH2_MASK       0x02U
#define MOTOR_PWM_CH3_MASK       0x04U
#define MOTOR_PWM_CH4_MASK       0x08U
#define MOTOR_BUTTON_DEBOUNCE_MS 50U
#define MOTOR_ENCODER_REPORT_MS  100U

static MotorControlState_t s_motor_state;
static uint16_t s_last_left_counter;
static uint16_t s_last_right_counter;
static int32_t s_left_window_delta;
static int32_t s_right_window_delta;
static uint8_t s_button_sample_mask;
static uint8_t s_button_stable_mask;
static uint8_t s_button_prev_stable_mask;
static uint32_t s_button_change_tick_ms;
static uint32_t s_last_encoder_report_tick_ms;

static bool MotorControl_StartChannel(uint32_t channel);
static bool MotorControl_ReadPc0(void);
static void MotorControl_UpdateEncoderDebug(void);
static uint8_t MotorControl_ReadLeftEncoderRaw(void);
static uint8_t MotorControl_ReadRightEncoderRaw(void);
static void MotorControl_ApplyRaw(uint16_t ch1_right_reverse,
                                  uint16_t ch2_right_forward,
                                  uint16_t ch3_left_forward,
                                  uint16_t ch4_left_reverse);
static int16_t MotorControl_ComputeDelta(uint16_t current, uint16_t previous);
static uint16_t MotorControl_PermilleToCompare(uint16_t duty_permille);

bool MotorControl_Init(void)
{
  bool ok = true;

  if (s_motor_state.ready)
  {
    return true;
  }

  ok = MotorControl_StartChannel(TIM_CHANNEL_1) && ok;
  ok = MotorControl_StartChannel(TIM_CHANNEL_2) && ok;
  ok = MotorControl_StartChannel(TIM_CHANNEL_3) && ok;
  ok = MotorControl_StartChannel(TIM_CHANNEL_4) && ok;

  s_last_left_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  s_last_right_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);
  s_motor_state.left_counter = s_last_left_counter;
  s_motor_state.right_counter = s_last_right_counter;
  s_motor_state.left_encoder_raw = MotorControl_ReadLeftEncoderRaw();
  s_motor_state.right_encoder_raw = MotorControl_ReadRightEncoderRaw();

  MotorControl_Stop();
  s_motor_state.ready = ok;
  return ok;
}

void MotorControl_Stop(void)
{
  MotorControl_ApplyRaw(0U, 0U, 0U, 0U);
  s_motor_state.forward_active = false;
  s_motor_state.mode = 0U;
  s_motor_state.duty_permille = 0U;
  s_motor_state.left_duty_permille = 0U;
  s_motor_state.right_duty_permille = 0U;
  s_motor_state.left_delta = 0;
  s_motor_state.right_delta = 0;
  s_motor_state.balance_error = 0;
  s_motor_state.correction_permille = 0;
  s_motor_state.active_pwm_mask = 0U;
  s_left_window_delta = 0;
  s_right_window_delta = 0;
}

void MotorControl_SetForward(uint16_t duty_permille)
{
  if (duty_permille > 1000U)
  {
    duty_permille = 1000U;
  }

  MotorControl_ApplyRaw(0U,
                        MotorControl_PermilleToCompare(duty_permille),
                        MotorControl_PermilleToCompare(duty_permille),
                        0U);
  s_motor_state.forward_active = (duty_permille > 0U);
  s_motor_state.mode = (duty_permille > 0U) ? 1U : 0U;
  s_motor_state.duty_permille = duty_permille;
  s_motor_state.left_duty_permille = duty_permille;
  s_motor_state.right_duty_permille = duty_permille;
  s_motor_state.correction_permille = 0;
  s_motor_state.active_pwm_mask = (duty_permille > 0U) ? (MOTOR_PWM_CH2_MASK | MOTOR_PWM_CH3_MASK) : 0U;
}

void MotorControl_UpdateButtons(void)
{
  uint8_t raw_mask;
  uint8_t pressed_edges;
  uint32_t now = HAL_GetTick();

  if (!s_motor_state.ready)
  {
    return;
  }

  raw_mask = MotorControl_ReadPc0() ? 0x01U : 0U;
  s_motor_state.button_mask = raw_mask;

  if (raw_mask != s_button_sample_mask)
  {
    s_button_sample_mask = raw_mask;
    s_button_change_tick_ms = now;
  }

  if ((now - s_button_change_tick_ms) >= MOTOR_BUTTON_DEBOUNCE_MS)
  {
    s_button_stable_mask = s_button_sample_mask;
  }

  pressed_edges = (uint8_t)(s_button_stable_mask & (uint8_t)~s_button_prev_stable_mask);
  if (pressed_edges != 0U)
  {
    if ((pressed_edges & 0x01U) != 0U)
    {
      if (s_motor_state.forward_active)
      {
        MotorControl_Stop();
      }
      else
      {
        s_last_left_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
        s_last_right_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);
        s_motor_state.left_counter = s_last_left_counter;
        s_motor_state.right_counter = s_last_right_counter;
        s_motor_state.left_delta = 0;
        s_motor_state.right_delta = 0;
        s_motor_state.balance_error = 0;
        s_left_window_delta = 0;
        s_right_window_delta = 0;
        s_last_encoder_report_tick_ms = now;
        MotorControl_SetForward(MOTOR_CAL_DUTY_PERMILLE);
      }
    }
  }
  s_button_prev_stable_mask = s_button_stable_mask;

  MotorControl_UpdateEncoderDebug();
}

bool MotorControl_GetState(MotorControlState_t *out_state)
{
  if (out_state == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *out_state = s_motor_state;
  taskEXIT_CRITICAL();
  return s_motor_state.ready;
}

static bool MotorControl_StartChannel(uint32_t channel)
{
  if (HAL_TIM_PWM_Start(&htim3, channel) != HAL_OK)
  {
    s_motor_state.start_errors++;
    return false;
  }

  return true;
}

static bool MotorControl_ReadPc0(void)
{
  return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET);
}

static void MotorControl_UpdateEncoderDebug(void)
{
  uint16_t left_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  uint16_t right_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);
  int16_t left_delta = MotorControl_ComputeDelta(left_now, s_last_left_counter);
  int16_t right_delta = MotorControl_ComputeDelta(right_now, s_last_right_counter);
  uint32_t now = HAL_GetTick();

  s_last_left_counter = left_now;
  s_last_right_counter = right_now;
  s_motor_state.left_counter = left_now;
  s_motor_state.right_counter = right_now;
  s_motor_state.left_encoder_raw = MotorControl_ReadLeftEncoderRaw();
  s_motor_state.right_encoder_raw = MotorControl_ReadRightEncoderRaw();
  s_left_window_delta += left_delta;
  s_right_window_delta += right_delta;
  if ((now - s_last_encoder_report_tick_ms) >= MOTOR_ENCODER_REPORT_MS)
  {
    s_motor_state.left_delta = (int16_t)s_left_window_delta;
    s_motor_state.right_delta = (int16_t)s_right_window_delta;
    s_motor_state.balance_error = s_left_window_delta - s_right_window_delta;
    s_left_window_delta = 0;
    s_right_window_delta = 0;
    s_last_encoder_report_tick_ms = now;
  }
  s_motor_state.control_ticks++;
}

static uint8_t MotorControl_ReadLeftEncoderRaw(void)
{
  uint8_t raw = 0U;

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) { raw |= 0x01U; }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) { raw |= 0x02U; }

  return raw;
}

static uint8_t MotorControl_ReadRightEncoderRaw(void)
{
  uint8_t raw = 0U;

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) { raw |= 0x01U; }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) { raw |= 0x02U; }

  return raw;
}

static void MotorControl_ApplyRaw(uint16_t ch1_right_reverse,
                                  uint16_t ch2_right_forward,
                                  uint16_t ch3_left_forward,
                                  uint16_t ch4_left_reverse)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ch1_right_reverse);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ch2_right_forward);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ch3_left_forward);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ch4_left_reverse);
}

static int16_t MotorControl_ComputeDelta(uint16_t current, uint16_t previous)
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

static uint16_t MotorControl_PermilleToCompare(uint16_t duty_permille)
{
  return (uint16_t)(((uint32_t)duty_permille * MOTOR_PWM_TIMER_MAX) / 1000U);
}
