################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: T0.axf

# Tool invocations
T0.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_board_nxp_lpcxpresso_824\Release" -L"C:\Users\mkurus\Desktop\backup\R2274ts\lpc_chip_82x\Release" -Xlinker -Map="T0.map" -Xlinker --gc-sections -flto -Os -mcpu=cortex-m0 -mthumb -T Trio_Custom.ld -o "T0.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) T0.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size T0.axf ; arm-none-eabi-objdump -S T0.axf >T0.lss ; arm-none-eabi-objcopy -O ihex T0.axf T0.hex ; arm-none-eabi-objcopy -O binary T0.axf T0.bin
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
