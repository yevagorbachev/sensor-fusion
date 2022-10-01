################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libraries/debug/debug.c 

OBJS += \
./Core/Libraries/debug/debug.o 

C_DEPS += \
./Core/Libraries/debug/debug.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libraries/debug/%.o Core/Libraries/debug/%.su: ../Core/Libraries/debug/%.c Core/Libraries/debug/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Libraries-2f-debug

clean-Core-2f-Libraries-2f-debug:
	-$(RM) ./Core/Libraries/debug/debug.d ./Core/Libraries/debug/debug.o ./Core/Libraries/debug/debug.su

.PHONY: clean-Core-2f-Libraries-2f-debug

