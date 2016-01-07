################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../startup/startup_stm32f334x8.S 

OBJS += \
./startup/startup_stm32f334x8.o 

S_UPPER_DEPS += \
./startup/startup_stm32f334x8.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F334C8Tx -DSTM32F3 -DSTM32F33 -DSTM32 -DSTM32F3348DISCOVERY -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F334x8 -IC:/Users/felipeneves/workspace/Disco_boost_converter/inc -IC:/Users/felipeneves/workspace/Disco_boost_converter/CMSIS/core -IC:/Users/felipeneves/workspace/Disco_boost_converter/CMSIS/device -IC:/Users/felipeneves/workspace/Disco_boost_converter/StdPeriph_Driver/inc -IC:/Users/felipeneves/workspace/Disco_boost_converter/Utilities -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


