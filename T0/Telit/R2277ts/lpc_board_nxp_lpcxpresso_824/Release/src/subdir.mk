################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/board.c \
../src/board_sysinit.c 

OBJS += \
./src/board.o \
./src/board_sysinit.o 

C_DEPS += \
./src/board.d \
./src/board_sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DNDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M0PLUS -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_chip_82x" -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_board_nxp_lpcxpresso_824" -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_chip_82x\inc" -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_board_nxp_lpcxpresso_824" -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_board_nxp_lpcxpresso_824\inc" -Os -g -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


