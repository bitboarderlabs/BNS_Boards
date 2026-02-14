################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
C_SRCS += \
../Core/Src/aio.c \
../Core/Src/app.c \
../Core/Src/board_id.c \
../Core/Src/bridge.c \
../Core/Src/comm.c \
../Core/Src/dio.c \
../Core/Src/main.c \
../Core/Src/modbus_tcp.c \
../Core/Src/node_id.c \
../Core/Src/packet.c \
../Core/Src/status_led.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/user_config.c \
../Core/Src/w5500.c \
../Core/Src/web_content.c \
../Core/Src/webserver.c

OBJS += \
./Core/Src/aio.o \
./Core/Src/app.o \
./Core/Src/board_id.o \
./Core/Src/bridge.o \
./Core/Src/comm.o \
./Core/Src/dio.o \
./Core/Src/main.o \
./Core/Src/modbus_tcp.o \
./Core/Src/node_id.o \
./Core/Src/packet.o \
./Core/Src/status_led.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/user_config.o \
./Core/Src/w5500.o \
./Core/Src/web_content.o \
./Core/Src/webserver.o

C_DEPS += \
./Core/Src/aio.d \
./Core/Src/app.d \
./Core/Src/board_id.d \
./Core/Src/bridge.d \
./Core/Src/comm.d \
./Core/Src/dio.d \
./Core/Src/main.d \
./Core/Src/modbus_tcp.d \
./Core/Src/node_id.d \
./Core/Src/packet.d \
./Core/Src/status_led.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/user_config.d \
./Core/Src/w5500.d \
./Core/Src/web_content.d \
./Core/Src/webserver.d


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DBOARD_BABY -DUSE_HAL_DRIVER -DSTM32F427xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/aio.cyclo ./Core/Src/aio.d ./Core/Src/aio.o ./Core/Src/aio.su ./Core/Src/app.cyclo ./Core/Src/app.d ./Core/Src/app.o ./Core/Src/app.su ./Core/Src/board_id.cyclo ./Core/Src/board_id.d ./Core/Src/board_id.o ./Core/Src/board_id.su ./Core/Src/bridge.cyclo ./Core/Src/bridge.d ./Core/Src/bridge.o ./Core/Src/bridge.su ./Core/Src/comm.cyclo ./Core/Src/comm.d ./Core/Src/comm.o ./Core/Src/comm.su ./Core/Src/dio.cyclo ./Core/Src/dio.d ./Core/Src/dio.o ./Core/Src/dio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/modbus_tcp.cyclo ./Core/Src/modbus_tcp.d ./Core/Src/modbus_tcp.o ./Core/Src/modbus_tcp.su ./Core/Src/node_id.cyclo ./Core/Src/node_id.d ./Core/Src/node_id.o ./Core/Src/node_id.su ./Core/Src/packet.cyclo ./Core/Src/packet.d ./Core/Src/packet.o ./Core/Src/packet.su ./Core/Src/status_led.cyclo ./Core/Src/status_led.d ./Core/Src/status_led.o ./Core/Src/status_led.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/user_config.cyclo ./Core/Src/user_config.d ./Core/Src/user_config.o ./Core/Src/user_config.su ./Core/Src/w5500.cyclo ./Core/Src/w5500.d ./Core/Src/w5500.o ./Core/Src/w5500.su ./Core/Src/web_content.cyclo ./Core/Src/web_content.d ./Core/Src/web_content.o ./Core/Src/web_content.su ./Core/Src/webserver.cyclo ./Core/Src/webserver.d ./Core/Src/webserver.o ./Core/Src/webserver.su

.PHONY: clean-Core-2f-Src
