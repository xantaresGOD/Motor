################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
device/%.obj: ../device/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu64 --idiv_support=idiv0 --tmu_support=tmu0 -Ooff --include_path="F:/DSP/CCS/ccs/tools/compiler/ti-cgt-c2000_20.2.0.LTS/include" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00_Software/utilities/clb_tool/clb_syscfg/systemc/include" --include_path="F:/DSP/CCS/controlSUITE/development_kits/BOOSTXL_POSMGR/v01_01_00_00/bissc-F28379DLpad-S2/Float/include" --include_path="F:/DSP/CCS/C2000Ware_3_03_00_00_Software/libraries/math/IQmath/c28/include" --include_path="F:/biss-c_workspace/svpwm/svpwm" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/device_support/f2838x/common/include" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/device_support/f2838x/headers/include" --include_path="F:/biss-c_workspace/svpwm/svpwm/device" --include_path="F:/DSP/CCS/C2000Ware_3_04_00_00/driverlib/f2838x/driverlib" --advice:performance=all --define=USE_25MHZ_XTAL --define=DEBUG --define=CPU1 --define=_DUAL_HEADERS --define=_FLASH --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="device/$(basename $(<F)).d_raw" --include_path="F:/biss-c_workspace/svpwm/svpwm/CPU1_FLASH/syscfg" --obj_directory="device" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


