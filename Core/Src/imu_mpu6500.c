#include "imu_mpu6500.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "app_platform.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MPU6500_REG_SMPLRT_DIV   0x19U
#define MPU6500_REG_CONFIG       0x1AU
#define MPU6500_REG_GYRO_CONFIG  0x1BU
#define MPU6500_REG_ACCEL_CONFIG 0x1CU
#define MPU6500_REG_PWR_MGMT_1   0x6BU
#define MPU6500_REG_WHO_AM_I     0x75U
#define MPU6500_REG_GYRO_ZOUT_H  0x47U

static imu_state_t g_imu_state;

static HAL_StatusTypeDef imu_mpu6500_write_reg(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status;
  uint8_t address = (uint8_t)(app_config_get()->imu_i2c_addr_7bit << 1);

  if (!app_platform_i2c_acquire(pdMS_TO_TICKS(10U)))
  {
    return HAL_TIMEOUT;
  }

  status = HAL_I2C_Mem_Write(app_platform_i2c(), address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, 50U);
  app_platform_i2c_release();
  return status;
}

static HAL_StatusTypeDef imu_mpu6500_read_regs(uint8_t reg, uint8_t *buffer, uint16_t length)
{
  HAL_StatusTypeDef status;
  uint8_t address = (uint8_t)(app_config_get()->imu_i2c_addr_7bit << 1);

  if ((buffer == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  if (!app_platform_i2c_acquire(pdMS_TO_TICKS(10U)))
  {
    return HAL_TIMEOUT;
  }

  status = HAL_I2C_Mem_Read(app_platform_i2c(), address, reg, I2C_MEMADD_SIZE_8BIT, buffer, length, 50U);
  app_platform_i2c_release();
  return status;
}

bool imu_mpu6500_init(void)
{
  uint8_t who_am_i = 0U;

  memset(&g_imu_state, 0, sizeof(g_imu_state));

  if (imu_mpu6500_read_regs(MPU6500_REG_WHO_AM_I, &who_am_i, 1U) != HAL_OK)
  {
    return false;
  }

  g_imu_state.who_am_i = who_am_i;
  g_imu_state.initialized = true;
  g_imu_state.healthy = ((who_am_i == 0x70U) || (who_am_i == 0x71U));

  if (imu_mpu6500_write_reg(MPU6500_REG_PWR_MGMT_1, 0x01U) != HAL_OK)
  {
    return false;
  }
  (void)imu_mpu6500_write_reg(MPU6500_REG_SMPLRT_DIV, 0x09U);
  (void)imu_mpu6500_write_reg(MPU6500_REG_CONFIG, 0x03U);
  (void)imu_mpu6500_write_reg(MPU6500_REG_GYRO_CONFIG, 0x18U);
  (void)imu_mpu6500_write_reg(MPU6500_REG_ACCEL_CONFIG, 0x00U);

  return true;
}

imu_state_t imu_mpu6500_poll(void)
{
  uint8_t data[2] = {0};
  int16_t raw_gz = 0;

  if (!g_imu_state.initialized)
  {
    return g_imu_state;
  }

  if (imu_mpu6500_read_regs(MPU6500_REG_GYRO_ZOUT_H, data, sizeof(data)) == HAL_OK)
  {
    raw_gz = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    g_imu_state.gyro_z_radps = ((float)raw_gz / 16.4f) * ((float)M_PI / 180.0f);
    g_imu_state.timestamp_ms = HAL_GetTick();
    if ((g_imu_state.who_am_i == 0x70U) || (g_imu_state.who_am_i == 0x71U))
    {
      g_imu_state.healthy = true;
    }
  }
  else
  {
    g_imu_state.healthy = false;
  }

  return g_imu_state;
}
