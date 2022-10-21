################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral_Driver/Src/oled.c 

OBJS += \
./Peripheral_Driver/Src/oled.o 

C_DEPS += \
./Peripheral_Driver/Src/oled.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral_Driver/Src/%.o Peripheral_Driver/Src/%.su: ../Peripheral_Driver/Src/%.c Peripheral_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/weeju/Documents/GitHub/MDP-Group14-Github/MDP/3002proj/Peripheral_Driver/Inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripheral_Driver-2f-Src

clean-Peripheral_Driver-2f-Src:
	-$(RM) ./Peripheral_Driver/Src/oled.d ./Peripheral_Driver/Src/oled.o ./Peripheral_Driver/Src/oled.su

.PHONY: clean-Peripheral_Driver-2f-Src

