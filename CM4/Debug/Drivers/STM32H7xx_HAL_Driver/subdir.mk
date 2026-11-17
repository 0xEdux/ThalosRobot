################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c 

OBJS += \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_dma_ex.o \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_exti.o \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash.o \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash_ex.o \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_i2c.o \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr.o \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr_ex.o \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_tim_ex.o \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_uart.o 

C_DEPS += \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_dma_ex.d \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_exti.d \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash.d \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash_ex.d \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_i2c.d \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr.d \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr_ex.d \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_tim_ex.d \
./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_uart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_dma_ex.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_exti.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash_ex.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_i2c.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr_ex.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_tim_ex.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_uart.o: C:/Users/juanb/OneDrive/Escritorio/ThalosRobot/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c Drivers/STM32H7xx_HAL_Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32H7xx_HAL_Driver

clean-Drivers-2f-STM32H7xx_HAL_Driver:
	-$(RM) ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_dma_ex.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_dma_ex.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_dma_ex.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_dma_ex.su ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_exti.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_exti.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_exti.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_exti.su ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash.su ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash_ex.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash_ex.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash_ex.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_flash_ex.su ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_i2c.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_i2c.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_i2c.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_i2c.su ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr.su ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr_ex.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr_ex.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr_ex.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_pwr_ex.su ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_tim_ex.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_tim_ex.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_tim_ex.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_tim_ex.su ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_uart.cyclo ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_uart.d ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_uart.o ./Drivers/STM32H7xx_HAL_Driver/stm32h7xx_hal_uart.su

.PHONY: clean-Drivers-2f-STM32H7xx_HAL_Driver

