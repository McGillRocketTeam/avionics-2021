################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Telemetry_Translation.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Telemetry_Translation.list \

OBJCOPY_BIN += \
Telemetry_Translation.bin \


# All Target
all: Telemetry_Translation.elf secondary-outputs

# Tool invocations
Telemetry_Translation.elf: $(OBJS) $(USER_OBJS) C:\Users\SlinderKid\Desktop\McGill\00\ -\ MRT\avionics-2021\Flight\ Computer\ R&D\Testing\Telemetry_Translation\STM32F303RETX_FLASH.ld
	arm-none-eabi-gcc -o "Telemetry_Translation.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\SlinderKid\Desktop\McGill\00 - MRT\avionics-2021\Flight Computer R&D\Testing\Telemetry_Translation\STM32F303RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Telemetry_Translation.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Telemetry_Translation.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Telemetry_Translation.list"
	@echo 'Finished building: $@'
	@echo ' '

Telemetry_Translation.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Telemetry_Translation.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
