################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/stm32f0-hal/stm32f0xx_hal.c \
../system/src/stm32f0-hal/stm32f0xx_hal_cortex.c \
../system/src/stm32f0-hal/stm32f0xx_hal_dma.c \
../system/src/stm32f0-hal/stm32f0xx_hal_flash.c \
../system/src/stm32f0-hal/stm32f0xx_hal_flash_ex.c \
../system/src/stm32f0-hal/stm32f0xx_hal_gpio.c \
../system/src/stm32f0-hal/stm32f0xx_hal_iwdg.c \
../system/src/stm32f0-hal/stm32f0xx_hal_pwr.c \
../system/src/stm32f0-hal/stm32f0xx_hal_pwr_ex.c \
../system/src/stm32f0-hal/stm32f0xx_hal_rcc.c \
../system/src/stm32f0-hal/stm32f0xx_hal_rcc_ex.c \
../system/src/stm32f0-hal/stm32f0xx_hal_spi.c \
../system/src/stm32f0-hal/stm32f0xx_hal_spi_ex.c \
../system/src/stm32f0-hal/stm32f0xx_hal_tim.c \
../system/src/stm32f0-hal/stm32f0xx_hal_tim_ex.c \
../system/src/stm32f0-hal/stm32l0xx_nucleo.c \
../system/src/stm32f0-hal/stm32xx_lpm.c 

OBJS += \
./system/src/stm32f0-hal/stm32f0xx_hal.o \
./system/src/stm32f0-hal/stm32f0xx_hal_cortex.o \
./system/src/stm32f0-hal/stm32f0xx_hal_dma.o \
./system/src/stm32f0-hal/stm32f0xx_hal_flash.o \
./system/src/stm32f0-hal/stm32f0xx_hal_flash_ex.o \
./system/src/stm32f0-hal/stm32f0xx_hal_gpio.o \
./system/src/stm32f0-hal/stm32f0xx_hal_iwdg.o \
./system/src/stm32f0-hal/stm32f0xx_hal_pwr.o \
./system/src/stm32f0-hal/stm32f0xx_hal_pwr_ex.o \
./system/src/stm32f0-hal/stm32f0xx_hal_rcc.o \
./system/src/stm32f0-hal/stm32f0xx_hal_rcc_ex.o \
./system/src/stm32f0-hal/stm32f0xx_hal_spi.o \
./system/src/stm32f0-hal/stm32f0xx_hal_spi_ex.o \
./system/src/stm32f0-hal/stm32f0xx_hal_tim.o \
./system/src/stm32f0-hal/stm32f0xx_hal_tim_ex.o \
./system/src/stm32f0-hal/stm32l0xx_nucleo.o \
./system/src/stm32f0-hal/stm32xx_lpm.o 

C_DEPS += \
./system/src/stm32f0-hal/stm32f0xx_hal.d \
./system/src/stm32f0-hal/stm32f0xx_hal_cortex.d \
./system/src/stm32f0-hal/stm32f0xx_hal_dma.d \
./system/src/stm32f0-hal/stm32f0xx_hal_flash.d \
./system/src/stm32f0-hal/stm32f0xx_hal_flash_ex.d \
./system/src/stm32f0-hal/stm32f0xx_hal_gpio.d \
./system/src/stm32f0-hal/stm32f0xx_hal_iwdg.d \
./system/src/stm32f0-hal/stm32f0xx_hal_pwr.d \
./system/src/stm32f0-hal/stm32f0xx_hal_pwr_ex.d \
./system/src/stm32f0-hal/stm32f0xx_hal_rcc.d \
./system/src/stm32f0-hal/stm32f0xx_hal_rcc_ex.d \
./system/src/stm32f0-hal/stm32f0xx_hal_spi.d \
./system/src/stm32f0-hal/stm32f0xx_hal_spi_ex.d \
./system/src/stm32f0-hal/stm32f0xx_hal_tim.d \
./system/src/stm32f0-hal/stm32f0xx_hal_tim_ex.d \
./system/src/stm32f0-hal/stm32l0xx_nucleo.d \
./system/src/stm32f0-hal/stm32xx_lpm.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/stm32f0-hal/%.o: ../system/src/stm32f0-hal/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F030 -DHSE_VALUE=8000000 -DUSE_HAL_DRIVER -DSTM32F030x6 -I"/home/jet/work/workspace/bt.relay/system/include/stm32f0-hal" -I"/home/jet/work/workspace/bt.relay/include" -I"/home/jet/work/workspace/bt.relay/system/include" -I"/home/jet/work/workspace/bt.relay/system/include/cmsis" -I"/home/jet/work/workspace/bt.relay/bluenrg/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


