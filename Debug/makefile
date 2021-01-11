################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./main.obj" \
"./startup_msp432p401r_ccs.obj" \
"./system_msp432p401r.obj" \
"./driverlib_files/adc14.obj" \
"./driverlib_files/aes256.obj" \
"./driverlib_files/comp_e.obj" \
"./driverlib_files/cpu.obj" \
"./driverlib_files/crc32.obj" \
"./driverlib_files/cs.obj" \
"./driverlib_files/dma.obj" \
"./driverlib_files/flash.obj" \
"./driverlib_files/fpu.obj" \
"./driverlib_files/gpio.obj" \
"./driverlib_files/i2c.obj" \
"./driverlib_files/interrupt.obj" \
"./driverlib_files/mpu.obj" \
"./driverlib_files/pcm.obj" \
"./driverlib_files/pmap.obj" \
"./driverlib_files/pss.obj" \
"./driverlib_files/ref_a.obj" \
"./driverlib_files/reset.obj" \
"./driverlib_files/rtc_c.obj" \
"./driverlib_files/spi.obj" \
"./driverlib_files/sysctl.obj" \
"./driverlib_files/systick.obj" \
"./driverlib_files/timer32.obj" \
"./driverlib_files/timer_a.obj" \
"./driverlib_files/uart.obj" \
"./driverlib_files/wdt_a.obj" \
"../msp432p401r.cmd" \
"../driverlib_files/ccs/msp432p4xx_driverlib.lib" \
"../driverlib_files/keil/msp432p4xx_driverlib.lib" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include driverlib_files/subdir_vars.mk
-include driverlib_files/ccs/subdir_vars.mk
-include driverlib_files/ewarm/subdir_vars.mk
-include driverlib_files/gcc/subdir_vars.mk
-include driverlib_files/keil/subdir_vars.mk
-include subdir_rules.mk
-include driverlib_files/subdir_rules.mk
-include driverlib_files/ccs/subdir_rules.mk
-include driverlib_files/ewarm/subdir_rules.mk
-include driverlib_files/gcc/subdir_rules.mk
-include driverlib_files/keil/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
GVSU_LakerRacing_Dashboard.out \

EXE_OUTPUTS__QUOTED += \
"GVSU_LakerRacing_Dashboard.out" \

BIN_OUTPUTS += \
GVSU_LakerRacing_Dashboard.hex \

BIN_OUTPUTS__QUOTED += \
"GVSU_LakerRacing_Dashboard.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(A_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "GVSU_LakerRacing_Dashboard.out"

# Tool invocations
GVSU_LakerRacing_Dashboard.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(A_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"GVSU_LakerRacing_Dashboard.map" --heap_size=1024 --stack_size=512 -i"C:/ti/ccsv8/ccs_base/arm/include" -i"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/lib" -i"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="GVSU_LakerRacing_Dashboard_linkInfo.xml" --rom_model -o "GVSU_LakerRacing_Dashboard.out" $(ORDERED_OBJS) $(A_SRCS)
	@echo 'Finished building target: "$@"'
	@echo ' '

GVSU_LakerRacing_Dashboard.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS/bin/armhex" --memwidth=8 --romwidth=8 -o "GVSU_LakerRacing_Dashboard.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.obj" "startup_msp432p401r_ccs.obj" "system_msp432p401r.obj" "driverlib_files\adc14.obj" "driverlib_files\aes256.obj" "driverlib_files\comp_e.obj" "driverlib_files\cpu.obj" "driverlib_files\crc32.obj" "driverlib_files\cs.obj" "driverlib_files\dma.obj" "driverlib_files\flash.obj" "driverlib_files\fpu.obj" "driverlib_files\gpio.obj" "driverlib_files\i2c.obj" "driverlib_files\interrupt.obj" "driverlib_files\mpu.obj" "driverlib_files\pcm.obj" "driverlib_files\pmap.obj" "driverlib_files\pss.obj" "driverlib_files\ref_a.obj" "driverlib_files\reset.obj" "driverlib_files\rtc_c.obj" "driverlib_files\spi.obj" "driverlib_files\sysctl.obj" "driverlib_files\systick.obj" "driverlib_files\timer32.obj" "driverlib_files\timer_a.obj" "driverlib_files\uart.obj" "driverlib_files\wdt_a.obj" 
	-$(RM) "main.d" "startup_msp432p401r_ccs.d" "system_msp432p401r.d" "driverlib_files\adc14.d" "driverlib_files\aes256.d" "driverlib_files\comp_e.d" "driverlib_files\cpu.d" "driverlib_files\crc32.d" "driverlib_files\cs.d" "driverlib_files\dma.d" "driverlib_files\flash.d" "driverlib_files\fpu.d" "driverlib_files\gpio.d" "driverlib_files\i2c.d" "driverlib_files\interrupt.d" "driverlib_files\mpu.d" "driverlib_files\pcm.d" "driverlib_files\pmap.d" "driverlib_files\pss.d" "driverlib_files\ref_a.d" "driverlib_files\reset.d" "driverlib_files\rtc_c.d" "driverlib_files\spi.d" "driverlib_files\sysctl.d" "driverlib_files\systick.d" "driverlib_files\timer32.d" "driverlib_files\timer_a.d" "driverlib_files\uart.d" "driverlib_files\wdt_a.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

