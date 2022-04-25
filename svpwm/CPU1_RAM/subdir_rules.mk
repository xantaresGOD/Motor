################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1559300063: ../MotorDriver.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"F:/DSP/CCS/ccs/utils/sysconfig_1.9.0/sysconfig_cli.bat" -s "F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/.metadata/product.json" --script "F:/biss-c_workspace/svpwm/svpwm/MotorDriver.syscfg" -o "syscfg" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/clb_config.h: build-1559300063 ../MotorDriver.syscfg
syscfg/clb_config.c: build-1559300063
syscfg/clb.dot: build-1559300063
syscfg/clb_sim.cpp: build-1559300063
syscfg/: build-1559300063

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 -Ooff --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/include" --include_path="F:/DSP/CCS/controlSUITE/libs/app_libs/position_manager/v01_01_00_00/bissc/Float/include" --include_path="F:/DSP/CCS/C2000Ware_3_03_00_00_Software/libraries/math/IQmath/c28/include" --include_path="F:/biss-c_workspace/svpwm/svpwm" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/device_support/f2838x/common/include" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/device_support/f2838x/headers/include" --include_path="F:/DSP/CCS/C2000Ware_MotorControl_SDK_3_02_00_00/solutions/boostxl_posmgr/f2838x/include" --include_path="F:/biss-c_workspace/svpwm/svpwm/device" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/driverlib/f2838x/driverlib" --include_path="F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/include" --advice:performance=all --define=_DUAL_HEADERS --define=DEBUG --define=CPU1 --define=USE_25MHZ_XTAL --relaxed_ansi --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/%.obj: ./syscfg/%.cpp $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 -Ooff --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/include" --include_path="F:/DSP/CCS/controlSUITE/libs/app_libs/position_manager/v01_01_00_00/bissc/Float/include" --include_path="F:/DSP/CCS/C2000Ware_3_03_00_00_Software/libraries/math/IQmath/c28/include" --include_path="F:/biss-c_workspace/svpwm/svpwm" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/device_support/f2838x/common/include" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/device_support/f2838x/headers/include" --include_path="F:/DSP/CCS/C2000Ware_MotorControl_SDK_3_02_00_00/solutions/boostxl_posmgr/f2838x/include" --include_path="F:/biss-c_workspace/svpwm/svpwm/device" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/driverlib/f2838x/driverlib" --include_path="F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/include" --advice:performance=all --define=_DUAL_HEADERS --define=DEBUG --define=CPU1 --define=USE_25MHZ_XTAL --relaxed_ansi --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 -Ooff --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/include" --include_path="F:/DSP/CCS/controlSUITE/libs/app_libs/position_manager/v01_01_00_00/bissc/Float/include" --include_path="F:/DSP/CCS/C2000Ware_3_03_00_00_Software/libraries/math/IQmath/c28/include" --include_path="F:/biss-c_workspace/svpwm/svpwm" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/device_support/f2838x/common/include" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/device_support/f2838x/headers/include" --include_path="F:/DSP/CCS/C2000Ware_MotorControl_SDK_3_02_00_00/solutions/boostxl_posmgr/f2838x/include" --include_path="F:/biss-c_workspace/svpwm/svpwm/device" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/driverlib/f2838x/driverlib" --include_path="F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/include" --advice:performance=all --define=_DUAL_HEADERS --define=DEBUG --define=CPU1 --define=USE_25MHZ_XTAL --relaxed_ansi --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="F:/biss-c_workspace/svpwm/svpwm/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


