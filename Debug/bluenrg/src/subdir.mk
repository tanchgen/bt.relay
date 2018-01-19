################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bluenrg/src/bluenrg_gap_aci.c \
../bluenrg/src/bluenrg_gatt_aci.c \
../bluenrg/src/bluenrg_hal_aci.c \
../bluenrg/src/bluenrg_itf_template.c \
../bluenrg/src/bluenrg_l2cap_aci.c \
../bluenrg/src/gp_timer.c \
../bluenrg/src/hci.c \
../bluenrg/src/list.c \
../bluenrg/src/osal.c \
../bluenrg/src/stm32_bluenrg_ble.c 

OBJS += \
./bluenrg/src/bluenrg_gap_aci.o \
./bluenrg/src/bluenrg_gatt_aci.o \
./bluenrg/src/bluenrg_hal_aci.o \
./bluenrg/src/bluenrg_itf_template.o \
./bluenrg/src/bluenrg_l2cap_aci.o \
./bluenrg/src/gp_timer.o \
./bluenrg/src/hci.o \
./bluenrg/src/list.o \
./bluenrg/src/osal.o \
./bluenrg/src/stm32_bluenrg_ble.o 

C_DEPS += \
./bluenrg/src/bluenrg_gap_aci.d \
./bluenrg/src/bluenrg_gatt_aci.d \
./bluenrg/src/bluenrg_hal_aci.d \
./bluenrg/src/bluenrg_itf_template.d \
./bluenrg/src/bluenrg_l2cap_aci.d \
./bluenrg/src/gp_timer.d \
./bluenrg/src/hci.d \
./bluenrg/src/list.d \
./bluenrg/src/osal.d \
./bluenrg/src/stm32_bluenrg_ble.d 


# Each subdirectory must supply rules for building sources it contributes
bluenrg/src/%.o: ../bluenrg/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F030 -DHSE_VALUE=8000000 -DUSE_HAL_DRIVER -DSTM32F030x6 -I"/home/jet/work/workspace/bt.relay/system/include/stm32f0-hal" -I"/home/jet/work/workspace/bt.relay/include" -I"/home/jet/work/workspace/bt.relay/system/include" -I"/home/jet/work/workspace/bt.relay/system/include/cmsis" -I"/home/jet/work/workspace/bt.relay/bluenrg/inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


