################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sensors/accel.c \
../Core/Src/sensors/gyro.c \
../Core/Src/sensors/i3g4250d_lib.c \
../Core/Src/sensors/lis3mdl_lib.c \
../Core/Src/sensors/lsm303agr_lib.c 

OBJS += \
./Core/Src/sensors/accel.o \
./Core/Src/sensors/gyro.o \
./Core/Src/sensors/i3g4250d_lib.o \
./Core/Src/sensors/lis3mdl_lib.o \
./Core/Src/sensors/lsm303agr_lib.o 

C_DEPS += \
./Core/Src/sensors/accel.d \
./Core/Src/sensors/gyro.d \
./Core/Src/sensors/i3g4250d_lib.d \
./Core/Src/sensors/lis3mdl_lib.d \
./Core/Src/sensors/lsm303agr_lib.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sensors/%.o Core/Src/sensors/%.su: ../Core/Src/sensors/%.c Core/Src/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sensors

clean-Core-2f-Src-2f-sensors:
	-$(RM) ./Core/Src/sensors/accel.d ./Core/Src/sensors/accel.o ./Core/Src/sensors/accel.su ./Core/Src/sensors/gyro.d ./Core/Src/sensors/gyro.o ./Core/Src/sensors/gyro.su ./Core/Src/sensors/i3g4250d_lib.d ./Core/Src/sensors/i3g4250d_lib.o ./Core/Src/sensors/i3g4250d_lib.su ./Core/Src/sensors/lis3mdl_lib.d ./Core/Src/sensors/lis3mdl_lib.o ./Core/Src/sensors/lis3mdl_lib.su ./Core/Src/sensors/lsm303agr_lib.d ./Core/Src/sensors/lsm303agr_lib.o ./Core/Src/sensors/lsm303agr_lib.su

.PHONY: clean-Core-2f-Src-2f-sensors

