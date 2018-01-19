################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/cmsis/system_stm32f0xx.c 

S_UPPER_SRCS += \
../system/src/cmsis/startup_stm32f030x6.S 

OBJS += \
./system/src/cmsis/startup_stm32f030x6.o \
./system/src/cmsis/system_stm32f0xx.o 

S_UPPER_DEPS += \
./system/src/cmsis/startup_stm32f030x6.d 

C_DEPS += \
./system/src/cmsis/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/cmsis/%.o: ../system/src/cmsis/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -x assembler-with-cpp -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F030 -DHSE_VALUE=8000000 -DUSE_HAL_DRIVER -DSTM32F030x6 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"/home/jet/work/workspace/bt.relay/bluenrg/inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

system/src/cmsis/%.o: ../system/src/cmsis/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F030 -DHSE_VALUE=8000000 -DUSE_HAL_DRIVER -DSTM32F030x6 -I"/home/jet/work/workspace/bt.relay/system/include/stm32f0-hal" -I"/home/jet/work/workspace/bt.relay/include" -I"/home/jet/work/workspace/bt.relay/system/include" -I"/home/jet/work/workspace/bt.relay/system/include/cmsis" -I"/home/jet/work/workspace/bt.relay/bluenrg/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


