################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Devices/lsm303agr/accel.c \
../Core/Devices/lsm303agr/lsm303agr_reg.c 

OBJS += \
./Core/Devices/lsm303agr/accel.o \
./Core/Devices/lsm303agr/lsm303agr_reg.o 

C_DEPS += \
./Core/Devices/lsm303agr/accel.d \
./Core/Devices/lsm303agr/lsm303agr_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Devices/lsm303agr/%.o Core/Devices/lsm303agr/%.su: ../Core/Devices/lsm303agr/%.c Core/Devices/lsm303agr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Devices-2f-lsm303agr

clean-Core-2f-Devices-2f-lsm303agr:
	-$(RM) ./Core/Devices/lsm303agr/accel.d ./Core/Devices/lsm303agr/accel.o ./Core/Devices/lsm303agr/accel.su ./Core/Devices/lsm303agr/lsm303agr_reg.d ./Core/Devices/lsm303agr/lsm303agr_reg.o ./Core/Devices/lsm303agr/lsm303agr_reg.su

.PHONY: clean-Core-2f-Devices-2f-lsm303agr

