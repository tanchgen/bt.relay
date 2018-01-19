################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/_write.c \
../src/bluenrg_interface.c \
../src/bt01.c \
../src/clock.c \
../src/main.c \
../src/my_function.c \
../src/my_service.c \
../src/stm32f0xx_hal_msp.c \
../src/stm32xx_it.c 

OBJS += \
./src/_write.o \
./src/bluenrg_interface.o \
./src/bt01.o \
./src/clock.o \
./src/main.o \
./src/my_function.o \
./src/my_service.o \
./src/stm32f0xx_hal_msp.o \
./src/stm32xx_it.o 

C_DEPS += \
./src/_write.d \
./src/bluenrg_interface.d \
./src/bt01.d \
./src/clock.d \
./src/main.d \
./src/my_function.d \
./src/my_service.d \
./src/stm32f0xx_hal_msp.d \
./src/stm32xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F030 -DHSE_VALUE=8000000 -DUSE_HAL_DRIVER -DSTM32F030x6 -I"/home/jet/work/workspace/bt.relay/system/include/stm32f0-hal" -I"/home/jet/work/workspace/bt.relay/include" -I"/home/jet/work/workspace/bt.relay/system/include" -I"/home/jet/work/workspace/bt.relay/system/include/cmsis" -I"/home/jet/work/workspace/bt.relay/bluenrg/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


