################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/boost.c \
../src/main.c \
../src/system_stm32f30x.c 

S_UPPER_SRCS += \
../src/dpl.S 

OBJS += \
./src/boost.o \
./src/dpl.o \
./src/main.o \
./src/system_stm32f30x.o 

S_UPPER_DEPS += \
./src/dpl.d 

C_DEPS += \
./src/boost.d \
./src/main.d \
./src/system_stm32f30x.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F334C8Tx -DSTM32F3 -DSTM32F33 -DSTM32 -DSTM32F3348DISCOVERY -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F334x8 -IC:/Users/felipeneves/workspace/Disco_boost_converter/inc -IC:/Users/felipeneves/workspace/Disco_boost_converter/CMSIS/core -IC:/Users/felipeneves/workspace/Disco_boost_converter/CMSIS/device -IC:/Users/felipeneves/workspace/Disco_boost_converter/StdPeriph_Driver/inc -IC:/Users/felipeneves/workspace/Disco_boost_converter/Utilities -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F334C8Tx -DSTM32F3 -DSTM32F33 -DSTM32 -DSTM32F3348DISCOVERY -DDEBUG -DUSE_STDPERIPH_DRIVER -DSTM32F334x8 -IC:/Users/felipeneves/workspace/Disco_boost_converter/inc -IC:/Users/felipeneves/workspace/Disco_boost_converter/CMSIS/core -IC:/Users/felipeneves/workspace/Disco_boost_converter/CMSIS/device -IC:/Users/felipeneves/workspace/Disco_boost_converter/StdPeriph_Driver/inc -IC:/Users/felipeneves/workspace/Disco_boost_converter/Utilities -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


