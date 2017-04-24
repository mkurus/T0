################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../src/aeabi_romdiv_patch.s \
../src/bootloader_asm.s 

C_SRCS += \
../src/MMA7455.c \
../src/MMA8652.c \
../src/T0_Project.c \
../src/bootloader.c \
../src/cr_startup_lpc82x.c \
../src/crp.c \
../src/gps.c \
../src/gsm.c \
../src/i2c.c \
../src/messages.c \
../src/mtb.c \
../src/settings.c \
../src/spi.c \
../src/status.c \
../src/sysinit.c \
../src/timer.c \
../src/utils.c \
../src/xmodem1k.c 

OBJS += \
./src/MMA7455.o \
./src/MMA8652.o \
./src/T0_Project.o \
./src/aeabi_romdiv_patch.o \
./src/bootloader.o \
./src/bootloader_asm.o \
./src/cr_startup_lpc82x.o \
./src/crp.o \
./src/gps.o \
./src/gsm.o \
./src/i2c.o \
./src/messages.o \
./src/mtb.o \
./src/settings.o \
./src/spi.o \
./src/status.o \
./src/sysinit.o \
./src/timer.o \
./src/utils.o \
./src/xmodem1k.o 

C_DEPS += \
./src/MMA7455.d \
./src/MMA8652.d \
./src/T0_Project.d \
./src/bootloader.d \
./src/cr_startup_lpc82x.d \
./src/crp.d \
./src/gps.d \
./src/gsm.d \
./src/i2c.d \
./src/messages.d \
./src/mtb.d \
./src/settings.d \
./src/spi.d \
./src/status.d \
./src/sysinit.d \
./src/timer.d \
./src/utils.d \
./src/xmodem1k.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DNDEBUG -D__USE_ROMDIVIDE -D__CODE_RED -DCORE_M0PLUS -D__MTB_DISABLE -D__MTB_BUFFER_SIZE=256 -D__USE_LPCOPEN -D__LPC82X__ -D_REDLIB_ -D__REDLIB__ -I"C:\Users\mkurus\Desktop\backup\R2274ts\T0\inc" -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_board_nxp_lpcxpresso_824\inc" -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_chip_82x\inc" -Os -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -flto -ffat-lto-objects -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -DNDEBUG -D__CODE_RED -DCORE_M0PLUS -D__USE_ROMDIVIDE -D__USE_LPCOPEN -D__LPC82X__ -D__REDLIB__ -I"C:\Users\mkurus\Desktop\backup\R2274ts\T0\inc" -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_board_nxp_lpcxpresso_824\inc" -I"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_chip_82x\inc" -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


