################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Displays/ST7735_Canvas/ST7735Canvas.c \
../Displays/ST7735_Canvas/fonts.c 

OBJS += \
./Displays/ST7735_Canvas/ST7735Canvas.o \
./Displays/ST7735_Canvas/fonts.o 

C_DEPS += \
./Displays/ST7735_Canvas/ST7735Canvas.d \
./Displays/ST7735_Canvas/fonts.d 


# Each subdirectory must supply rules for building sources it contributes
Displays/ST7735_Canvas/%.o Displays/ST7735_Canvas/%.su Displays/ST7735_Canvas/%.cyclo: ../Displays/ST7735_Canvas/%.c Displays/ST7735_Canvas/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DBOARD_MAMA -DUSE_HAL_DRIVER -DSTM32F427xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/Alex.ST-EL/source/BNS_Boards/Displays" -I"C:/Users/Alex.ST-EL/source/BNS_Boards/Displays/Fonts" -I"C:/Users/Alex.ST-EL/source/BNS_Boards/Displays/ST7735_Canvas" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Displays-2f-ST7735_Canvas

clean-Displays-2f-ST7735_Canvas:
	-$(RM) ./Displays/ST7735_Canvas/ST7735Canvas.cyclo ./Displays/ST7735_Canvas/ST7735Canvas.d ./Displays/ST7735_Canvas/ST7735Canvas.o ./Displays/ST7735_Canvas/ST7735Canvas.su ./Displays/ST7735_Canvas/fonts.cyclo ./Displays/ST7735_Canvas/fonts.d ./Displays/ST7735_Canvas/fonts.o ./Displays/ST7735_Canvas/fonts.su

.PHONY: clean-Displays-2f-ST7735_Canvas

