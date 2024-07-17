################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Devices/lib/ftoa.c 

C_DEPS += \
./Core/Devices/lib/ftoa.d 

OBJS += \
./Core/Devices/lib/ftoa.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Devices/lib/%.o Core/Devices/lib/%.su Core/Devices/lib/%.cyclo: ../Core/Devices/lib/%.c Core/Devices/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Devices -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Devices-2f-lib

clean-Core-2f-Devices-2f-lib:
	-$(RM) ./Core/Devices/lib/ftoa.cyclo ./Core/Devices/lib/ftoa.d ./Core/Devices/lib/ftoa.o ./Core/Devices/lib/ftoa.su

.PHONY: clean-Core-2f-Devices-2f-lib

