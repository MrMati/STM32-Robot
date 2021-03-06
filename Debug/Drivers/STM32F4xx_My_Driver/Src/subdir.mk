################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c.c \
../Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c_ex.c \
../Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_spi.c 

OBJS += \
./Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c.o \
./Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c_ex.o \
./Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_spi.o 

C_DEPS += \
./Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c.d \
./Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c_ex.d \
./Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c.o: ../Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/BSP/Components" -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/STM32F4xx_My_Driver/Inc" -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/BSP/STM32F411E-Discovery" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c_ex.o: ../Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c_ex.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/BSP/Components" -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/STM32F4xx_My_Driver/Inc" -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/BSP/STM32F411E-Discovery" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_i2c_ex.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_spi.o: ../Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_spi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/BSP/Components" -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/STM32F4xx_My_Driver/Inc" -I"D:/Dokumenty/STM32CubeMX/Robot/Drivers/BSP/STM32F411E-Discovery" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32F4xx_My_Driver/Src/stm32f4xx_hal_spi.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

