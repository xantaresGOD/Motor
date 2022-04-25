################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./syscfg/clb_config.obj" \
"./syscfg/clb_sim.obj" \
"./f2838x_codestartbranch.obj" \
"./f2838x_usdelay.obj" \
"./device/device.obj" \
"./source/eqep_ex2_calculation.obj" \
"./source/f2838x_adc.obj" \
"./source/f2838x_cputimers.obj" \
"./source/f2838x_defaultisr.obj" \
"./source/f2838x_epwm.obj" \
"./source/f2838x_globalvariabledefs.obj" \
"./source/f2838x_gpio.obj" \
"./source/f2838x_piectrl.obj" \
"./source/f2838x_pievect.obj" \
"./source/f2838x_sysctrl.obj" \
"./source/program.obj" \
"./source/program2.obj" \
"./source/speed_observer.obj" \
"./source/svpwm_main.obj" \
"../2838x_RAM_lnk_cpu1.cmd" \
"F:/DSP/CCS/C2000Ware_3_04_00_00_Software/libraries/math/IQmath/c28/lib/IQmath_fpu32.lib" \
"F:/DSP/CCS/C2000Ware_3_04_00_00_Software/driverlib/f2838x/driverlib/ccs/Debug/driverlib.lib" \
"../f2838x_headers_nonBIOS_cpu1.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include device/subdir_vars.mk
-include source/subdir_vars.mk
-include subdir_rules.mk
-include device/subdir_rules.mk
-include source/subdir_rules.mk
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
MotorDriver.out \

EXE_OUTPUTS__QUOTED += \
"MotorDriver.out" \

BIN_OUTPUTS += \
MotorDriver.hex \

BIN_OUTPUTS__QUOTED += \
"MotorDriver.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "MotorDriver.out"

# Tool invocations
MotorDriver.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 -Ooff --advice:performance=all --define=_DUAL_HEADERS --define=DEBUG --define=CPU1 --define=USE_25MHZ_XTAL --relaxed_ansi --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi -z -m"MotorDriver.map" --stack_size=0x100 --warn_sections -i"F:/biss-c_workspace/Float/lib" -i"F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/lib" -i"F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/include" --reread_libs --diag_wrap=off --display_error_number --xml_link_info="MotorDriver_linkInfo.xml" --entry_point=code_start --rom_model -o "MotorDriver.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

MotorDriver.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: C2000 Hex Utility'
	"F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/bin/hex2000" --diag_wrap=off -o "MotorDriver.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "syscfg\clb_config.obj" "syscfg\clb_sim.obj" "f2838x_codestartbranch.obj" "f2838x_usdelay.obj" "device\device.obj" "source\eqep_ex2_calculation.obj" "source\f2838x_adc.obj" "source\f2838x_cputimers.obj" "source\f2838x_defaultisr.obj" "source\f2838x_epwm.obj" "source\f2838x_globalvariabledefs.obj" "source\f2838x_gpio.obj" "source\f2838x_piectrl.obj" "source\f2838x_pievect.obj" "source\f2838x_sysctrl.obj" "source\program.obj" "source\program2.obj" "source\speed_observer.obj" "source\svpwm_main.obj" 
	-$(RM) "syscfg\clb_config.d" "device\device.d" "source\eqep_ex2_calculation.d" "source\f2838x_adc.d" "source\f2838x_cputimers.d" "source\f2838x_defaultisr.d" "source\f2838x_epwm.d" "source\f2838x_globalvariabledefs.d" "source\f2838x_gpio.d" "source\f2838x_piectrl.d" "source\f2838x_pievect.d" "source\f2838x_sysctrl.d" "source\program.d" "source\program2.d" "source\speed_observer.d" "source\svpwm_main.d" 
	-$(RM) "syscfg\clb_sim.d" 
	-$(RM) "f2838x_codestartbranch.d" "f2838x_usdelay.d" 
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-mkdir "F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/simulation"
	-C:/TDM-GCC-64/bin/g++  -c -DCLB_SIM -IF:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc-2.3.3/src -IF:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/include/type3 -IF:/biss-c_workspace/svpwm/svpwm -IC:/TDM-GCC-64/bin/include -Og -g -gdwarf-3 -gstrict-dwarf -Wall -MMD -MP -MFF:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/simulation/clb_sim.d -MTF:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/simulation/clb_sim.o -IF:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/syscfg   -fno-threadsafe-statics  -oF:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/simulation/clb_sim.o F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/syscfg/clb_sim.cpp
	-C:/TDM-GCC-64/bin/g++ -DCLB_SIM -Og -g -gdwarf-3 -gstrict-dwarf -Wall -Wl,-Map,F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/simulation/simulation_output.map -LF:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc-2.3.3/build/src -oF:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/simulation/simulation_output.exe F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/simulation/clb_sim.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_FSM_SC_model.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_HLC_SC_model.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_LUT4_SC_model.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_OutputLUT_SC_model.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_counter_SC_model.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_counter_top_SC_model.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_FSM_top_SC_model.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_LUT4_top_SC_model.o F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/src/type3/CLB_OutputLUT_top_SC_model.o  -Wl,--start-group -lsystemc -Wl,--end-group
	-./simulation/simulation_output.exe
	-F:/DSP/CCS/ccs/tools/node/node.exe "F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/dot_file_libraries/clbDotUtility.js"  "F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg" "F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/syscfg" "F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/syscfg/clb.dot"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

