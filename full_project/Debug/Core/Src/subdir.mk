################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMP180.c \
../Core/Src/BUZZER.c \
../Core/Src/File_Handling.c \
../Core/Src/INA219.c \
../Core/Src/LED.c \
../Core/Src/MPU6050.c \
../Core/Src/SD_Handler.c \
../Core/Src/fatfs_sd_card.c \
../Core/Src/main.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/BMP180.o \
./Core/Src/BUZZER.o \
./Core/Src/File_Handling.o \
./Core/Src/INA219.o \
./Core/Src/LED.o \
./Core/Src/MPU6050.o \
./Core/Src/SD_Handler.o \
./Core/Src/fatfs_sd_card.o \
./Core/Src/main.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/BMP180.d \
./Core/Src/BUZZER.d \
./Core/Src/File_Handling.d \
./Core/Src/INA219.d \
./Core/Src/LED.d \
./Core/Src/MPU6050.d \
./Core/Src/SD_Handler.d \
./Core/Src/fatfs_sd_card.d \
./Core/Src/main.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Core/dd_sd_card -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMP180.cyclo ./Core/Src/BMP180.d ./Core/Src/BMP180.o ./Core/Src/BMP180.su ./Core/Src/BUZZER.cyclo ./Core/Src/BUZZER.d ./Core/Src/BUZZER.o ./Core/Src/BUZZER.su ./Core/Src/File_Handling.cyclo ./Core/Src/File_Handling.d ./Core/Src/File_Handling.o ./Core/Src/File_Handling.su ./Core/Src/INA219.cyclo ./Core/Src/INA219.d ./Core/Src/INA219.o ./Core/Src/INA219.su ./Core/Src/LED.cyclo ./Core/Src/LED.d ./Core/Src/LED.o ./Core/Src/LED.su ./Core/Src/MPU6050.cyclo ./Core/Src/MPU6050.d ./Core/Src/MPU6050.o ./Core/Src/MPU6050.su ./Core/Src/SD_Handler.cyclo ./Core/Src/SD_Handler.d ./Core/Src/SD_Handler.o ./Core/Src/SD_Handler.su ./Core/Src/fatfs_sd_card.cyclo ./Core/Src/fatfs_sd_card.d ./Core/Src/fatfs_sd_card.o ./Core/Src/fatfs_sd_card.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

