#include "buttons.h"

#include "main.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  button_event_t event;
  uint8_t stable_level;
  uint8_t last_sample;
  uint32_t last_change_ms;
} button_channel_t;

static button_channel_t g_buttons[] =
{
  {BUTTON1_START_GPIO_Port, BUTTON1_START_Pin, BUTTON_EVENT_START, 1U, 1U, 0U},
  {BUTTON2_STOP_GPIO_Port, BUTTON2_STOP_Pin, BUTTON_EVENT_STOP, 1U, 1U, 0U},
  {BUTTON3_RETURN_GPIO_Port, BUTTON3_RETURN_Pin, BUTTON_EVENT_RETURN, 1U, 1U, 0U},
  {BUTTON4_PAGE_GPIO_Port, BUTTON4_PAGE_Pin, BUTTON_EVENT_PAGE, 1U, 1U, 0U},
};

void buttons_init(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t index;

  for (index = 0U; index < (sizeof(g_buttons) / sizeof(g_buttons[0])); ++index)
  {
    const GPIO_PinState level = HAL_GPIO_ReadPin(g_buttons[index].port, g_buttons[index].pin);
    g_buttons[index].stable_level = (uint8_t)level;
    g_buttons[index].last_sample = (uint8_t)level;
    g_buttons[index].last_change_ms = now;
  }
}

button_event_t buttons_poll(void)
{
  const uint32_t now = HAL_GetTick();
  uint32_t index;

  for (index = 0U; index < (sizeof(g_buttons) / sizeof(g_buttons[0])); ++index)
  {
    const uint8_t sample = (uint8_t)HAL_GPIO_ReadPin(g_buttons[index].port, g_buttons[index].pin);

    if (sample != g_buttons[index].last_sample)
    {
      g_buttons[index].last_sample = sample;
      g_buttons[index].last_change_ms = now;
    }

    if ((sample != g_buttons[index].stable_level) && ((now - g_buttons[index].last_change_ms) >= 30U))
    {
      g_buttons[index].stable_level = sample;
      if (sample == (uint8_t)GPIO_PIN_RESET)
      {
        return g_buttons[index].event;
      }
    }
  }

  return BUTTON_EVENT_NONE;
}
