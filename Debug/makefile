################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include fcb-source/utilities/src/subdir.mk
-include fcb-source/sensors/src/subdir.mk
-include fcb-source/nanopb-0.3.3-windows-x86/subdir.mk
-include fcb-source/fcb/src/subdir.mk
-include fcb-source/fcb-drivers/BSP/STM32F3-Discovery/subdir.mk
-include fcb-source/fcb-drivers/BSP/Components/lsm303dlhc/subdir.mk
-include fcb-source/fcb-drivers/BSP/Components/l3gd20/subdir.mk
-include fcb-source/communication/usb-cdc-com/src/subdir.mk
-include fcb-source/communication/protobuf/subdir.mk
-include fcb-source/cmsis-boot/startup/subdir.mk
-include fcb-source/STM32_USB_Device_Library/Core/Src/subdir.mk
-include fcb-source/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include fcb-source/STM32F3xx_HAL_Driver/Src/subdir.mk
-include fcb-source/FreeRTOS/Source/portable/MemMang/subdir.mk
-include fcb-source/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include fcb-source/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include fcb-source/FreeRTOS/Source/subdir.mk
-include fcb-source/FreeRTOS-Plus/Source/FreeRTOS-Plus-CLI/subdir.mk
-include fcb-source/CMSIS/Device/ST/STM32F3xx/Source/Templates/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/TransformFunctions/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/SupportFunctions/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/StatisticsFunctions/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/MatrixFunctions/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/FilteringFunctions/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/FastMathFunctions/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/ControllerFunctions/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/ComplexMathFunctions/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/CommonTables/subdir.mk
-include fcb-source/CMSIS/DSP_Lib/Source/BasicMathFunctions/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
dragonfly-fcb.hex \

SECONDARY_SIZE += \
dragonfly-fcb.siz \


# All Target
all: dragonfly-fcb.elf secondary-outputs

# Tool invocations
dragonfly-fcb.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -T "C:\Users\A501632\workspace\dragonfly-fcb\fcb-source\ldscripts\arm-gcc-link.ld" -nostartfiles -Xlinker --gc-sections -L"C:\Users\A501632\workspace\dragonfly-fcb\fcb-source\CMSIS\Lib\GCC" -Wl,-Map,"dragonfly-fcb.map" --specs=nano.specs -u _printf_float -u _scanf_float -specs=nosys.specs -o "dragonfly-fcb.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

dragonfly-fcb.hex: dragonfly-fcb.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "dragonfly-fcb.elf"  "dragonfly-fcb.hex"
	@echo 'Finished building: $@'
	@echo ' '

dragonfly-fcb.siz: dragonfly-fcb.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "dragonfly-fcb.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(OBJS)$(C_UPPER_DEPS)$(CXX_DEPS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS)$(CPP_DEPS) dragonfly-fcb.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
