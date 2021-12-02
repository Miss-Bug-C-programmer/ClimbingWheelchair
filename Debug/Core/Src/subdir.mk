################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/PID.c \
../Core/Src/X2_6010S.c \
../Core/Src/adc.c \
../Core/Src/battery.c \
../Core/Src/bd25l.c \
../Core/Src/button.c \
../Core/Src/can.c \
../Core/Src/differentialDrive.c \
../Core/Src/dma.c \
../Core/Src/dwt_delay.c \
../Core/Src/encoder.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/joystick.c \
../Core/Src/mpu6050.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/test_battery.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/wheelchair.c 

OBJS += \
./Core/Src/PID.o \
./Core/Src/X2_6010S.o \
./Core/Src/adc.o \
./Core/Src/battery.o \
./Core/Src/bd25l.o \
./Core/Src/button.o \
./Core/Src/can.o \
./Core/Src/differentialDrive.o \
./Core/Src/dma.o \
./Core/Src/dwt_delay.o \
./Core/Src/encoder.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/joystick.o \
./Core/Src/mpu6050.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/test_battery.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/wheelchair.o 

C_DEPS += \
./Core/Src/PID.d \
./Core/Src/X2_6010S.d \
./Core/Src/adc.d \
./Core/Src/battery.d \
./Core/Src/bd25l.d \
./Core/Src/button.d \
./Core/Src/can.d \
./Core/Src/differentialDrive.d \
./Core/Src/dma.d \
./Core/Src/dwt_delay.d \
./Core/Src/encoder.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/joystick.d \
./Core/Src/mpu6050.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/test_battery.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/wheelchair.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

