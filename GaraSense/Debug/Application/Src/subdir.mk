################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/Src/ao.c \
../Application/Src/ao_broker.c \
../Application/Src/ao_display.c \
../Application/Src/ao_distance.c \
../Application/Src/ao_env.c \
../Application/Src/ao_logger.c 

OBJS += \
./Application/Src/ao.o \
./Application/Src/ao_broker.o \
./Application/Src/ao_display.o \
./Application/Src/ao_distance.o \
./Application/Src/ao_env.o \
./Application/Src/ao_logger.o 

C_DEPS += \
./Application/Src/ao.d \
./Application/Src/ao_broker.d \
./Application/Src/ao_display.d \
./Application/Src/ao_distance.d \
./Application/Src/ao_env.d \
./Application/Src/ao_logger.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Src/%.o Application/Src/%.su Application/Src/%.cyclo: ../Application/Src/%.c Application/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-Src

clean-Application-2f-Src:
	-$(RM) ./Application/Src/ao.cyclo ./Application/Src/ao.d ./Application/Src/ao.o ./Application/Src/ao.su ./Application/Src/ao_broker.cyclo ./Application/Src/ao_broker.d ./Application/Src/ao_broker.o ./Application/Src/ao_broker.su ./Application/Src/ao_display.cyclo ./Application/Src/ao_display.d ./Application/Src/ao_display.o ./Application/Src/ao_display.su ./Application/Src/ao_distance.cyclo ./Application/Src/ao_distance.d ./Application/Src/ao_distance.o ./Application/Src/ao_distance.su ./Application/Src/ao_env.cyclo ./Application/Src/ao_env.d ./Application/Src/ao_env.o ./Application/Src/ao_env.su ./Application/Src/ao_logger.cyclo ./Application/Src/ao_logger.d ./Application/Src/ao_logger.o ./Application/Src/ao_logger.su

.PHONY: clean-Application-2f-Src

