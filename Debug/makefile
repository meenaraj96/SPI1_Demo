################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Driver/Src/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := SPI1_Demo2_Digital_Waveform
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
SPI1_Demo2_Digital_Waveform.elf \

MAP_FILES += \
SPI1_Demo2_Digital_Waveform.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SPI1_Demo2_Digital_Waveform.list \


# All Target
all: main-build

# Main-build Target
main-build: SPI1_Demo2_Digital_Waveform.elf secondary-outputs

# Tool invocations
SPI1_Demo2_Digital_Waveform.elf SPI1_Demo2_Digital_Waveform.map: $(OBJS) $(USER_OBJS) D:\Study_Materials\Embedded\ Systems\Pantech\ Solutions\STM32CubeIDE\Workspace\ Programs\TARGET\SPI1_Demo2_Digital_Waveform\STM32F411RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "SPI1_Demo2_Digital_Waveform.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Study_Materials\Embedded Systems\Pantech Solutions\STM32CubeIDE\Workspace Programs\TARGET\SPI1_Demo2_Digital_Waveform\STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SPI1_Demo2_Digital_Waveform.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SPI1_Demo2_Digital_Waveform.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SPI1_Demo2_Digital_Waveform.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) SPI1_Demo2_Digital_Waveform.elf SPI1_Demo2_Digital_Waveform.list SPI1_Demo2_Digital_Waveform.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
