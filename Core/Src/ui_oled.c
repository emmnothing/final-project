#include "ui_oled.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "app_platform.h"
#include "app_state.h"

typedef struct
{
  bool initialized;
  uint8_t buffer[APP_OLED_BUFFER_SIZE];
} ui_oled_ctx_t;

static ui_oled_ctx_t g_oled;

static const uint8_t FONT_DIGITS[10][5] =
{
  {0x3E, 0x51, 0x49, 0x45, 0x3E},
  {0x00, 0x42, 0x7F, 0x40, 0x00},
  {0x42, 0x61, 0x51, 0x49, 0x46},
  {0x21, 0x41, 0x45, 0x4B, 0x31},
  {0x18, 0x14, 0x12, 0x7F, 0x10},
  {0x27, 0x45, 0x45, 0x45, 0x39},
  {0x3C, 0x4A, 0x49, 0x49, 0x30},
  {0x01, 0x71, 0x09, 0x05, 0x03},
  {0x36, 0x49, 0x49, 0x49, 0x36},
  {0x06, 0x49, 0x49, 0x29, 0x1E},
};

static const uint8_t FONT_UPPER[26][5] =
{
  {0x7E, 0x11, 0x11, 0x11, 0x7E},
  {0x7F, 0x49, 0x49, 0x49, 0x36},
  {0x3E, 0x41, 0x41, 0x41, 0x22},
  {0x7F, 0x41, 0x41, 0x22, 0x1C},
  {0x7F, 0x49, 0x49, 0x49, 0x41},
  {0x7F, 0x09, 0x09, 0x09, 0x01},
  {0x3E, 0x41, 0x49, 0x49, 0x3A},
  {0x7F, 0x08, 0x08, 0x08, 0x7F},
  {0x00, 0x41, 0x7F, 0x41, 0x00},
  {0x20, 0x40, 0x41, 0x3F, 0x01},
  {0x7F, 0x08, 0x14, 0x22, 0x41},
  {0x7F, 0x40, 0x40, 0x40, 0x40},
  {0x7F, 0x02, 0x0C, 0x02, 0x7F},
  {0x7F, 0x04, 0x08, 0x10, 0x7F},
  {0x3E, 0x41, 0x41, 0x41, 0x3E},
  {0x7F, 0x09, 0x09, 0x09, 0x06},
  {0x3E, 0x41, 0x51, 0x21, 0x5E},
  {0x7F, 0x09, 0x19, 0x29, 0x46},
  {0x46, 0x49, 0x49, 0x49, 0x31},
  {0x01, 0x01, 0x7F, 0x01, 0x01},
  {0x3F, 0x40, 0x40, 0x40, 0x3F},
  {0x1F, 0x20, 0x40, 0x20, 0x1F},
  {0x3F, 0x40, 0x38, 0x40, 0x3F},
  {0x63, 0x14, 0x08, 0x14, 0x63},
  {0x07, 0x08, 0x70, 0x08, 0x07},
  {0x61, 0x51, 0x49, 0x45, 0x43},
};

static bool ui_oled_write_command(uint8_t command)
{
  uint8_t packet[2] = {0x00U, command};
  HAL_StatusTypeDef status;

  if (!app_platform_i2c_acquire(pdMS_TO_TICKS(10U)))
  {
    return false;
  }
  status = HAL_I2C_Master_Transmit(app_platform_i2c(),
                                   (uint16_t)(app_config_get()->oled_i2c_addr_7bit << 1),
                                   packet,
                                   sizeof(packet),
                                   50U);
  app_platform_i2c_release();
  return (status == HAL_OK);
}

static bool ui_oled_write_data(const uint8_t *data, uint16_t length)
{
  uint8_t packet[17];
  uint16_t offset = 0U;

  if (data == NULL)
  {
    return false;
  }

  if (!app_platform_i2c_acquire(pdMS_TO_TICKS(10U)))
  {
    return false;
  }

  while (offset < length)
  {
    const uint16_t chunk = ((length - offset) > 16U) ? 16U : (length - offset);

    packet[0] = 0x40U;
    memcpy(&packet[1], &data[offset], chunk);
    if (HAL_I2C_Master_Transmit(app_platform_i2c(),
                                (uint16_t)(app_config_get()->oled_i2c_addr_7bit << 1),
                                packet,
                                (uint16_t)(chunk + 1U),
                                50U) != HAL_OK)
    {
      app_platform_i2c_release();
      return false;
    }
    offset += chunk;
  }

  app_platform_i2c_release();
  return true;
}

static const uint8_t *ui_oled_get_glyph(char c)
{
  static const uint8_t blank[5] = {0, 0, 0, 0, 0};
  static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
  static const uint8_t dot[5] = {0x00, 0x60, 0x60, 0x00, 0x00};
  static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
  static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
  static const uint8_t equal[5] = {0x14, 0x14, 0x14, 0x14, 0x14};
  static const uint8_t lp[5] = {0x00, 0x1C, 0x22, 0x41, 0x00};
  static const uint8_t rp[5] = {0x00, 0x41, 0x22, 0x1C, 0x00};

  if ((c >= 'a') && (c <= 'z'))
  {
    c = (char)(c - ('a' - 'A'));
  }

  if ((c >= '0') && (c <= '9'))
  {
    return FONT_DIGITS[(uint32_t)(c - '0')];
  }
  if ((c >= 'A') && (c <= 'Z'))
  {
    return FONT_UPPER[(uint32_t)(c - 'A')];
  }

  switch (c)
  {
    case '-':
      return dash;
    case '.':
      return dot;
    case ':':
      return colon;
    case '/':
      return slash;
    case '=':
      return equal;
    case '(':
      return lp;
    case ')':
      return rp;
    case ' ':
    default:
      return blank;
  }
}

static void ui_oled_clear_buffer(void)
{
  memset(g_oled.buffer, 0, sizeof(g_oled.buffer));
}

static void ui_oled_draw_char(uint8_t x, uint8_t row, char c)
{
  const uint8_t *glyph = ui_oled_get_glyph(c);
  uint32_t offset;
  uint32_t i;

  if ((row >= (APP_OLED_HEIGHT / 8U)) || (x >= APP_OLED_WIDTH))
  {
    return;
  }

  offset = ((uint32_t)row * APP_OLED_WIDTH) + x;
  for (i = 0U; (i < 5U) && ((offset + i) < sizeof(g_oled.buffer)); ++i)
  {
    g_oled.buffer[offset + i] = glyph[i];
  }
}

static void ui_oled_draw_text(uint8_t x, uint8_t row, const char *text)
{
  while ((text != NULL) && (*text != '\0') && (x < (APP_OLED_WIDTH - 5U)))
  {
    ui_oled_draw_char(x, row, *text);
    x = (uint8_t)(x + 6U);
    ++text;
  }
}

static bool ui_oled_flush(void)
{
  uint8_t page;

  for (page = 0U; page < (APP_OLED_HEIGHT / 8U); ++page)
  {
    if (!ui_oled_write_command((uint8_t)(0xB0U + page)) ||
        !ui_oled_write_command(0x00U) ||
        !ui_oled_write_command(0x10U) ||
        !ui_oled_write_data(&g_oled.buffer[page * APP_OLED_WIDTH], APP_OLED_WIDTH))
    {
      return false;
    }
  }

  return true;
}

bool ui_oled_init(void)
{
  static const uint8_t init_sequence[] =
  {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0x8DU, 0x14U, 0x20U, 0x00U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
    0x81U, 0x8FU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
    0x2EU, 0xAFU
  };
  uint32_t index;

  for (index = 0U; index < (sizeof(init_sequence) / sizeof(init_sequence[0])); ++index)
  {
    if (!ui_oled_write_command(init_sequence[index]))
    {
      g_oled.initialized = false;
      return false;
    }
  }

  g_oled.initialized = true;
  ui_oled_clear_buffer();
  ui_oled_draw_text(0U, 0U, "BOOT");
  (void)ui_oled_flush();
  return true;
}

void ui_oled_render(void)
{
  char line[24];
  const app_snapshot_t snapshot = app_state_get_snapshot();

  if (!g_oled.initialized)
  {
    return;
  }

  ui_oled_clear_buffer();

  if (snapshot.oled_page == 0U)
  {
    (void)snprintf(line, sizeof(line), "MODE %s", app_state_mode_string(snapshot.mode));
    ui_oled_draw_text(0U, 0U, line);
    (void)snprintf(line, sizeof(line), "FLT %08lX", (unsigned long)snapshot.faults);
    ui_oled_draw_text(0U, 1U, line);
    (void)snprintf(line, sizeof(line), "SPD %.2f", (double)snapshot.params.speed_max_mps);
    ui_oled_draw_text(0U, 2U, line);
    (void)snprintf(line, sizeof(line), "TRN %.2f", (double)snapshot.params.turn_max_radps);
    ui_oled_draw_text(0U, 3U, line);
    (void)snprintf(line, sizeof(line), "SAFE %.2f", (double)snapshot.params.safety_distance_m);
    ui_oled_draw_text(0U, 4U, line);
    (void)snprintf(line, sizeof(line), "ADC %u %u %u",
                   snapshot.params.raw_adc[0],
                   snapshot.params.raw_adc[1],
                   snapshot.params.raw_adc[2]);
    ui_oled_draw_text(0U, 5U, line);
    ui_oled_draw_text(0U, 7U, "PAGE 0");
  }
  else
  {
    (void)snprintf(line, sizeof(line), "X %.2f Y %.2f", (double)snapshot.pose.x_m, (double)snapshot.pose.y_m);
    ui_oled_draw_text(0U, 0U, line);
    (void)snprintf(line, sizeof(line), "YAW %.2f", (double)snapshot.pose.yaw_rad);
    ui_oled_draw_text(0U, 1U, line);
    (void)snprintf(line, sizeof(line), "WL %.1f WR %.1f",
                   (double)snapshot.wheels.left_ticks_per_s,
                   (double)snapshot.wheels.right_ticks_per_s);
    ui_oled_draw_text(0U, 2U, line);
    (void)snprintf(line, sizeof(line), "LDR %lu %lu",
                   (unsigned long)snapshot.lidar.bytes_received,
                   (unsigned long)snapshot.lidar.frames_parsed);
    ui_oled_draw_text(0U, 3U, line);
    (void)snprintf(line, sizeof(line), "IMU %02X %.2f",
                   (unsigned int)snapshot.imu.who_am_i,
                   (double)snapshot.imu.gyro_z_radps);
    ui_oled_draw_text(0U, 4U, line);
    (void)snprintf(line, sizeof(line), "BT %s", snapshot.bt_link_active ? "ON" : "OFF");
    ui_oled_draw_text(0U, 5U, line);
    ui_oled_draw_text(0U, 7U, "PAGE 1");
  }

  if (!ui_oled_flush())
  {
    g_oled.initialized = false;
  }
}
