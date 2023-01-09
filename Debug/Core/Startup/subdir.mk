################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l053xxGCC.s \
../Core/Startup/stm32l0xx_STLRamMcMxGCC.s \
../Core/Startup/stm32l0xx_STLcpurunGCC.s \
../Core/Startup/stm32l0xx_STLcpustartGCC.s 

OBJS += \
./Core/Startup/startup_stm32l053xxGCC.o \
./Core/Startup/stm32l0xx_STLRamMcMxGCC.o \
./Core/Startup/stm32l0xx_STLcpurunGCC.o \
./Core/Startup/stm32l0xx_STLcpustartGCC.o 

S_DEPS += \
./Core/Startup/startup_stm32l053xxGCC.d \
./Core/Startup/stm32l0xx_STLRamMcMxGCC.d \
./Core/Startup/stm32l0xx_STLcpurunGCC.d \
./Core/Startup/stm32l0xx_STLcpustartGCC.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -I"C:/Users/strngr/STM32CubeIDE/workspace_1.11.0/test/Middlewares/STM32_SelfTest_Library/inc" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l053xxGCC.d ./Core/Startup/startup_stm32l053xxGCC.o ./Core/Startup/stm32l0xx_STLRamMcMxGCC.d ./Core/Startup/stm32l0xx_STLRamMcMxGCC.o ./Core/Startup/stm32l0xx_STLcpurunGCC.d ./Core/Startup/stm32l0xx_STLcpurunGCC.o ./Core/Startup/stm32l0xx_STLcpustartGCC.d ./Core/Startup/stm32l0xx_STLcpustartGCC.o

.PHONY: clean-Core-2f-Startup

