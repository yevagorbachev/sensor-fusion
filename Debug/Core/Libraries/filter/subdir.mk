################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libraries/filter/filter.c 

OBJS += \
./Core/Libraries/filter/filter.o 

C_DEPS += \
./Core/Libraries/filter/filter.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libraries/filter/%.o Core/Libraries/filter/%.su: ../Core/Libraries/filter/%.c Core/Libraries/filter/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Libraries-2f-filter

clean-Core-2f-Libraries-2f-filter:
	-$(RM) ./Core/Libraries/filter/filter.d ./Core/Libraries/filter/filter.o ./Core/Libraries/filter/filter.su

.PHONY: clean-Core-2f-Libraries-2f-filter

