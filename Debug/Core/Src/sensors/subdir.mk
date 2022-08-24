################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sensors/accel.c \
../Core/Src/sensors/gyro.c \
../Core/Src/sensors/i3g4250d_reg.c \
../Core/Src/sensors/lis3mdl_reg.c \
../Core/Src/sensors/lsm303agr_reg.c \
../Core/Src/sensors/mag_e.c \
../Core/Src/sensors/mag_i.c 

OBJS += \
./Core/Src/sensors/accel.o \
./Core/Src/sensors/gyro.o \
./Core/Src/sensors/i3g4250d_reg.o \
./Core/Src/sensors/lis3mdl_reg.o \
./Core/Src/sensors/lsm303agr_reg.o \
./Core/Src/sensors/mag_e.o \
./Core/Src/sensors/mag_i.o 

C_DEPS += \
./Core/Src/sensors/accel.d \
./Core/Src/sensors/gyro.d \
./Core/Src/sensors/i3g4250d_reg.d \
./Core/Src/sensors/lis3mdl_reg.d \
./Core/Src/sensors/lsm303agr_reg.d \
./Core/Src/sensors/mag_e.d \
./Core/Src/sensors/mag_i.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sensors/%.o Core/Src/sensors/%.su: ../Core/Src/sensors/%.c Core/Src/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sensors

clean-Core-2f-Src-2f-sensors:
	-$(RM) ./Core/Src/sensors/accel.d ./Core/Src/sensors/accel.o ./Core/Src/sensors/accel.su ./Core/Src/sensors/gyro.d ./Core/Src/sensors/gyro.o ./Core/Src/sensors/gyro.su ./Core/Src/sensors/i3g4250d_reg.d ./Core/Src/sensors/i3g4250d_reg.o ./Core/Src/sensors/i3g4250d_reg.su ./Core/Src/sensors/lis3mdl_reg.d ./Core/Src/sensors/lis3mdl_reg.o ./Core/Src/sensors/lis3mdl_reg.su ./Core/Src/sensors/lsm303agr_reg.d ./Core/Src/sensors/lsm303agr_reg.o ./Core/Src/sensors/lsm303agr_reg.su ./Core/Src/sensors/mag_e.d ./Core/Src/sensors/mag_e.o ./Core/Src/sensors/mag_e.su ./Core/Src/sensors/mag_i.d ./Core/Src/sensors/mag_i.o ./Core/Src/sensors/mag_i.su

.PHONY: clean-Core-2f-Src-2f-sensors

