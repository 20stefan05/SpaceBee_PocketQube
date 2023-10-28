################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/SD_Module/SD_Handler.c \
../Core/Src/SD_Module/fatfs_sd_card.c 

OBJS += \
./Core/Src/SD_Module/SD_Handler.o \
./Core/Src/SD_Module/fatfs_sd_card.o 

C_DEPS += \
./Core/Src/SD_Module/SD_Handler.d \
./Core/Src/SD_Module/fatfs_sd_card.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/SD_Module/%.o Core/Src/SD_Module/%.su Core/Src/SD_Module/%.cyclo: ../Core/Src/SD_Module/%.c Core/Src/SD_Module/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-SD_Module

clean-Core-2f-Src-2f-SD_Module:
	-$(RM) ./Core/Src/SD_Module/SD_Handler.cyclo ./Core/Src/SD_Module/SD_Handler.d ./Core/Src/SD_Module/SD_Handler.o ./Core/Src/SD_Module/SD_Handler.su ./Core/Src/SD_Module/fatfs_sd_card.cyclo ./Core/Src/SD_Module/fatfs_sd_card.d ./Core/Src/SD_Module/fatfs_sd_card.o ./Core/Src/SD_Module/fatfs_sd_card.su

.PHONY: clean-Core-2f-Src-2f-SD_Module

