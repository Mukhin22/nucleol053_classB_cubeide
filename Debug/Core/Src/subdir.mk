################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Src/stm32l0xx_STLRamMcMxGCC.s \
../Core/Src/stm32l0xx_STLcpurunGCC.s \
../Core/Src/stm32l0xx_STLcpustartGCC.s 

C_SRCS += \
../Core/Src/main.c \
../Core/Src/stm32fxx_STLclockrun.c \
../Core/Src/stm32fxx_STLclockstart.c \
../Core/Src/stm32fxx_STLcrc32Run.c \
../Core/Src/stm32fxx_STLmain.c \
../Core/Src/stm32fxx_STLstartup.c \
../Core/Src/stm32fxx_STLtranspRam.c \
../Core/Src/stm32l0xx_hal_msp.c \
../Core/Src/stm32l0xx_it.c \
../Core/Src/stm32l0xx_nucleo.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l0xx.c 

OBJS += \
./Core/Src/main.o \
./Core/Src/stm32fxx_STLclockrun.o \
./Core/Src/stm32fxx_STLclockstart.o \
./Core/Src/stm32fxx_STLcrc32Run.o \
./Core/Src/stm32fxx_STLmain.o \
./Core/Src/stm32fxx_STLstartup.o \
./Core/Src/stm32fxx_STLtranspRam.o \
./Core/Src/stm32l0xx_STLRamMcMxGCC.o \
./Core/Src/stm32l0xx_STLcpurunGCC.o \
./Core/Src/stm32l0xx_STLcpustartGCC.o \
./Core/Src/stm32l0xx_hal_msp.o \
./Core/Src/stm32l0xx_it.o \
./Core/Src/stm32l0xx_nucleo.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l0xx.o 

S_DEPS += \
./Core/Src/stm32l0xx_STLRamMcMxGCC.d \
./Core/Src/stm32l0xx_STLcpurunGCC.d \
./Core/Src/stm32l0xx_STLcpustartGCC.d 

C_DEPS += \
./Core/Src/main.d \
./Core/Src/stm32fxx_STLclockrun.d \
./Core/Src/stm32fxx_STLclockstart.d \
./Core/Src/stm32fxx_STLcrc32Run.d \
./Core/Src/stm32fxx_STLmain.d \
./Core/Src/stm32fxx_STLstartup.d \
./Core/Src/stm32fxx_STLtranspRam.d \
./Core/Src/stm32l0xx_hal_msp.d \
./Core/Src/stm32l0xx_it.d \
./Core/Src/stm32l0xx_nucleo.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-strict-aliasing -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.s Core/Src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -DUSE_MARCHX_TEST -DARTISAN -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

