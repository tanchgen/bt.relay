################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/include/stm32f0-hal/stm32xx_lpm.c 

OBJS += \
./system/include/stm32f0-hal/stm32xx_lpm.o 

C_DEPS += \
./system/include/stm32f0-hal/stm32xx_lpm.d 


# Each subdirectory must supply rules for building sources it contributes
system/include/stm32f0-hal/%.o: ../system/include/stm32f0-hal/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F030 -DHSE_VALUE=8000000 -DUSE_HAL_DRIVER -DSTM32F030x6 -I"/home/jet/work/workspace/bt.relay/system/include/stm32f0-hal" -I"/home/jet/work/workspace/bt.relay/include" -I"/home/jet/work/workspace/bt.relay/system/include" -I"/home/jet/work/workspace/bt.relay/system/include/cmsis" -I"/home/jet/work/workspace/bt.relay/bluenrg/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


