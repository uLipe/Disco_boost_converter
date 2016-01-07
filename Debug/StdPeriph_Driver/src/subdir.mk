################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../StdPeriph_Driver/src/stm32f30x_adc.c \
../StdPeriph_Driver/src/stm32f30x_can.c \
../StdPeriph_Driver/src/stm32f30x_comp.c \
../StdPeriph_Driver/src/stm32f30x_crc.c \
../StdPeriph_Driver/src/stm32f30x_dac.c \
../StdPeriph_Driver/src/stm32f30x_dbgmcu.c \
../StdPeriph_Driver/src/stm32f30x_dma.c \
../StdPeriph_Driver/src/stm32f30x_exti.c \
../StdPeriph_Driver/src/stm32f30x_flash.c \
../StdPeriph_Driver/src/stm32f30x_gpio.c \
../StdPeriph_Driver/src/stm32f30x_hrtim.c \
../StdPeriph_Driver/src/stm32f30x_i2c.c \
../StdPeriph_Driver/src/stm32f30x_iwdg.c \
../StdPeriph_Driver/src/stm32f30x_misc.c \
../StdPeriph_Driver/src/stm32f30x_opamp.c \
../StdPeriph_Driver/src/stm32f30x_pwr.c \
../StdPeriph_Driver/src/stm32f30x_rcc.c \
../StdPeriph_Driver/src/stm32f30x_rtc.c \
../StdPeriph_Driver/src/stm32f30x_spi.c \
../StdPeriph_Driver/src/stm32f30x_syscfg.c \
../StdPeriph_Driver/src/stm32f30x_tim.c \
../StdPeriph_Driver/src/stm32f30x_usart.c \
../StdPeriph_Driver/src/stm32f30x_wwdg.c 

OBJS += \
./StdPeriph_Driver/src/stm32f30x_adc.o \
./StdPeriph_Driver/src/stm32f30x_can.o \
./StdPeriph_Driver/src/stm32f30x_comp.o \
./StdPeriph_Driver/src/stm32f30x_crc.o \
./StdPeriph_Driver/src/stm32f30x_dac.o \
./StdPeriph_Driver/src/stm32f30x_dbgmcu.o \
./StdPeriph_Driver/src/stm32f30x_dma.o \
./StdPeriph_Driver/src/stm32f30x_exti.o \
./StdPeriph_Driver/src/stm32f30x_flash.o \
./StdPeriph_Driver/src/stm32f30x_gpio.o \
./StdPeriph_Driver/src/stm32f30x_hrtim.o \
./StdPeriph_Driver/src/stm32f30x_i2c.o \
./StdPeriph_Driver/src/stm32f30x_iwdg.o \
./StdPeriph_Driver/src/stm32f30x_misc.o \
./StdPeriph_Driver/src/stm32f30x_opamp.o \
./StdPeriph_Driver/src/stm32f30x_pwr.o \
./StdPeriph_Driver/src/stm32f30x_rcc.o \
./StdPeriph_Driver/src/stm32f30x_rtc.o \
./StdPeriph_Driver/src/stm32f30x_spi.o \
./StdPeriph_Driver/src/stm32f30x_syscfg.o \
./StdPeriph_Driver/src/stm32f30x_tim.o \
./StdPeriph_Driver/src/stm32f30x_usart.o \
./StdPeriph_Driver/src/stm32f30x_wwdg.o 

C_DEPS += \
./StdPeriph_Driver/src/stm32f30x_adc.d \
./StdPeriph_Driver/src/stm32f30x_can.d \
./StdPeriph_Driver/src/stm32f30x_comp.d \
./StdPeriph_Driver/src/stm32f30x_crc.d \
./StdPeriph_Driver/src/stm32f30x_dac.d \
./StdPeriph_Driver/src/stm32f30x_dbgmcu.d \
./StdPeriph_Driver/src/stm32f30x_dma.d \
./StdPeriph_Driver/src/stm32f30x_exti.d \
./StdPeriph_Driver/src/stm32f30x_flash.d \
./StdPeriph_Driver/src/stm32f30x_gpio.d \
./StdPeriph_Driver/src/stm32f30x_hrtim.d \
./StdPeriph_Driver/src/stm32f30x_i2c.d \
./StdPeriph_Driver/src/stm32f30x_iwdg.d \
./StdPeriph_Driver/src/stm32f30x_misc.d \
./StdPeriph_Driver/src/stm32f30x_opamp.d \
./StdPeriph_Driver/src/stm32f30x_pwr.d \
./StdPeriph_Driver/src/stm32f30x_rcc.d \
./StdPeriph_Driver/src/stm32f30x_rtc.d \
./StdPeriph_Driver/src/stm32f30x_spi.d \
./StdPeriph_Driver/src/stm32f30x_syscfg.d \
./StdPeriph_Driver/src/stm32f30x_tim.d \
./StdPeriph_Driver/src/stm32f30x_usart.d \
./StdPeriph_Driver/src/stm32f30x_wwdg.d 


# Each subdirectory must supply rules for building sources it contributes
StdPeriph_Driver/src/%.o: ../StdPeriph_Driver/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F334C8Tx -DSTM32F3 -DSTM32F33 -DSTM32 -DSTM32F3348DISCOVERY -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F334x8 -IC:/Users/felipeneves/workspace/Disco_boost_converter/inc -IC:/Users/felipeneves/workspace/Disco_boost_converter/CMSIS/core -IC:/Users/felipeneves/workspace/Disco_boost_converter/CMSIS/device -IC:/Users/felipeneves/workspace/Disco_boost_converter/StdPeriph_Driver/inc -IC:/Users/felipeneves/workspace/Disco_boost_converter/Utilities -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


