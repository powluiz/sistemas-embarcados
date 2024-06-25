################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS-Plus-CLI/FreeRTOS_CLI.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS-Plus-CLI/FreeRTOS_CLI.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS-Plus-CLI/FreeRTOS_CLI.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS-Plus-CLI/%.o Middlewares/Third_Party/FreeRTOS-Plus-CLI/%.su Middlewares/Third_Party/FreeRTOS-Plus-CLI/%.cyclo: ../Middlewares/Third_Party/FreeRTOS-Plus-CLI/%.c Middlewares/Third_Party/FreeRTOS-Plus-CLI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/luiz1/STM32Cube/Repository/STM32Cube_FW_G4_V1.5.2/Drivers/CMSIS/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2d-Plus-2d-CLI

clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2d-Plus-2d-CLI:
	-$(RM) ./Middlewares/Third_Party/FreeRTOS-Plus-CLI/FreeRTOS_CLI.cyclo ./Middlewares/Third_Party/FreeRTOS-Plus-CLI/FreeRTOS_CLI.d ./Middlewares/Third_Party/FreeRTOS-Plus-CLI/FreeRTOS_CLI.o ./Middlewares/Third_Party/FreeRTOS-Plus-CLI/FreeRTOS_CLI.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-FreeRTOS-2d-Plus-2d-CLI

