################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Devices/lis3mdl/lis3mdl_reg.c \
../Core/Devices/lis3mdl/mag.c 

OBJS += \
./Core/Devices/lis3mdl/lis3mdl_reg.o \
./Core/Devices/lis3mdl/mag.o 

C_DEPS += \
./Core/Devices/lis3mdl/lis3mdl_reg.d \
./Core/Devices/lis3mdl/mag.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Devices/lis3mdl/%.o Core/Devices/lis3mdl/%.su: ../Core/Devices/lis3mdl/%.c Core/Devices/lis3mdl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Devices-2f-lis3mdl

clean-Core-2f-Devices-2f-lis3mdl:
	-$(RM) ./Core/Devices/lis3mdl/lis3mdl_reg.d ./Core/Devices/lis3mdl/lis3mdl_reg.o ./Core/Devices/lis3mdl/lis3mdl_reg.su ./Core/Devices/lis3mdl/mag.d ./Core/Devices/lis3mdl/mag.o ./Core/Devices/lis3mdl/mag.su

.PHONY: clean-Core-2f-Devices-2f-lis3mdl

