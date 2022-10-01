################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Libraries/sched/badsched.c 

OBJS += \
./Core/Libraries/sched/badsched.o 

C_DEPS += \
./Core/Libraries/sched/badsched.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libraries/sched/%.o Core/Libraries/sched/%.su: ../Core/Libraries/sched/%.c Core/Libraries/sched/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Libraries-2f-sched

clean-Core-2f-Libraries-2f-sched:
	-$(RM) ./Core/Libraries/sched/badsched.d ./Core/Libraries/sched/badsched.o ./Core/Libraries/sched/badsched.su

.PHONY: clean-Core-2f-Libraries-2f-sched

