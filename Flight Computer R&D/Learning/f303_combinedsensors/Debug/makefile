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
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
f303_combinedsensors.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
f303_combinedsensors.list \

OBJCOPY_BIN += \
f303_combinedsensors.bin \


# All Target
all: f303_combinedsensors.elf secondary-outputs

# Tool invocations
f303_combinedsensors.elf: $(OBJS) $(USER_OBJS) C:\Users\holyc\Documents\MRT\Flight\ Computer\ RnD\Tuts\f303_combinedsensors\STM32F303RETX_FLASH.ld
	arm-none-eabi-gcc -o "f303_combinedsensors.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\holyc\Documents\MRT\Flight Computer RnD\Tuts\f303_combinedsensors\STM32F303RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="f303_combinedsensors.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

f303_combinedsensors.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "f303_combinedsensors.list"
	@echo 'Finished building: $@'
	@echo ' '

f303_combinedsensors.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "f303_combinedsensors.bin"
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
