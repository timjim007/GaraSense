################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/ssd1306/fonts.c \
../ECUAL/ssd1306/ssd1306.c \
../ECUAL/ssd1306/ssd1306_cfg.c 

OBJS += \
./ECUAL/ssd1306/fonts.o \
./ECUAL/ssd1306/ssd1306.o \
./ECUAL/ssd1306/ssd1306_cfg.o 

C_DEPS += \
./ECUAL/ssd1306/fonts.d \
./ECUAL/ssd1306/ssd1306.d \
./ECUAL/ssd1306/ssd1306_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/ssd1306/%.o ECUAL/ssd1306/%.su ECUAL/ssd1306/%.cyclo: ../ECUAL/ssd1306/%.c ECUAL/ssd1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ECUAL-2f-ssd1306

clean-ECUAL-2f-ssd1306:
	-$(RM) ./ECUAL/ssd1306/fonts.cyclo ./ECUAL/ssd1306/fonts.d ./ECUAL/ssd1306/fonts.o ./ECUAL/ssd1306/fonts.su ./ECUAL/ssd1306/ssd1306.cyclo ./ECUAL/ssd1306/ssd1306.d ./ECUAL/ssd1306/ssd1306.o ./ECUAL/ssd1306/ssd1306.su ./ECUAL/ssd1306/ssd1306_cfg.cyclo ./ECUAL/ssd1306/ssd1306_cfg.d ./ECUAL/ssd1306/ssd1306_cfg.o ./ECUAL/ssd1306/ssd1306_cfg.su

.PHONY: clean-ECUAL-2f-ssd1306

