################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_config.c \
../Core/Src/app_platform.c \
../Core/Src/app_state.c \
../Core/Src/app_tasks.c \
../Core/Src/buttons.c \
../Core/Src/encoder.c \
../Core/Src/freertos.c \
../Core/Src/imu_mpu6500.c \
../Core/Src/lidar.c \
../Core/Src/main.c \
../Core/Src/motor_control.c \
../Core/Src/odometry.c \
../Core/Src/safety.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/telemetry_bt.c \
../Core/Src/ui_oled.c 

OBJS += \
./Core/Src/app_config.o \
./Core/Src/app_platform.o \
./Core/Src/app_state.o \
./Core/Src/app_tasks.o \
./Core/Src/buttons.o \
./Core/Src/encoder.o \
./Core/Src/freertos.o \
./Core/Src/imu_mpu6500.o \
./Core/Src/lidar.o \
./Core/Src/main.o \
./Core/Src/motor_control.o \
./Core/Src/odometry.o \
./Core/Src/safety.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/telemetry_bt.o \
./Core/Src/ui_oled.o 

C_DEPS += \
./Core/Src/app_config.d \
./Core/Src/app_platform.d \
./Core/Src/app_state.d \
./Core/Src/app_tasks.d \
./Core/Src/buttons.d \
./Core/Src/encoder.d \
./Core/Src/freertos.d \
./Core/Src/imu_mpu6500.d \
./Core/Src/lidar.d \
./Core/Src/main.d \
./Core/Src/motor_control.d \
./Core/Src/odometry.d \
./Core/Src/safety.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/telemetry_bt.d \
./Core/Src/ui_oled.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app_config.cyclo ./Core/Src/app_config.d ./Core/Src/app_config.o ./Core/Src/app_config.su ./Core/Src/app_platform.cyclo ./Core/Src/app_platform.d ./Core/Src/app_platform.o ./Core/Src/app_platform.su ./Core/Src/app_state.cyclo ./Core/Src/app_state.d ./Core/Src/app_state.o ./Core/Src/app_state.su ./Core/Src/app_tasks.cyclo ./Core/Src/app_tasks.d ./Core/Src/app_tasks.o ./Core/Src/app_tasks.su ./Core/Src/buttons.cyclo ./Core/Src/buttons.d ./Core/Src/buttons.o ./Core/Src/buttons.su ./Core/Src/encoder.cyclo ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/imu_mpu6500.cyclo ./Core/Src/imu_mpu6500.d ./Core/Src/imu_mpu6500.o ./Core/Src/imu_mpu6500.su ./Core/Src/lidar.cyclo ./Core/Src/lidar.d ./Core/Src/lidar.o ./Core/Src/lidar.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor_control.cyclo ./Core/Src/motor_control.d ./Core/Src/motor_control.o ./Core/Src/motor_control.su ./Core/Src/odometry.cyclo ./Core/Src/odometry.d ./Core/Src/odometry.o ./Core/Src/odometry.su ./Core/Src/safety.cyclo ./Core/Src/safety.d ./Core/Src/safety.o ./Core/Src/safety.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/telemetry_bt.cyclo ./Core/Src/telemetry_bt.d ./Core/Src/telemetry_bt.o ./Core/Src/telemetry_bt.su ./Core/Src/ui_oled.cyclo ./Core/Src/ui_oled.d ./Core/Src/ui_oled.o ./Core/Src/ui_oled.su

.PHONY: clean-Core-2f-Src

