################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Devices/BNO055.cpp 

OBJS += \
./Core/Devices/BNO055.o 

CPP_DEPS += \
./Core/Devices/BNO055.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Devices/%.o Core/Devices/%.su Core/Devices/%.cyclo: ../Core/Devices/%.cpp Core/Devices/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Devices -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Devices

clean-Core-2f-Devices:
	-$(RM) ./Core/Devices/BNO055.cyclo ./Core/Devices/BNO055.d ./Core/Devices/BNO055.o ./Core/Devices/BNO055.su

.PHONY: clean-Core-2f-Devices

