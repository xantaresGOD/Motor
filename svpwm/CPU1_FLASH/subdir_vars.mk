################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
./syscfg/clb_sim.cpp 

CMD_SRCS += \
../2838x_FLASH_lnk_cpu1.cmd \
../f2838x_headers_nonBIOS_cpu1.cmd 

SYSCFG_SRCS += \
../MotorDriver.syscfg 

LIB_SRCS += \
F:/DSP/CCS/C2000Ware_3_04_00_00_Software/libraries/math/IQmath/c28/lib/IQmath_fpu32.lib \
F:/DSP/CCS/C2000Ware_3_04_00_00_Software/driverlib/f2838x/driverlib/ccs/Debug/driverlib.lib 

ASM_SRCS += \
../f2838x_codestartbranch.asm \
../f2838x_usdelay.asm 

C_SRCS += \
./syscfg/clb_config.c 

GEN_FILES += \
./syscfg/clb_config.c \
./syscfg/clb_sim.cpp 

GEN_MISC_DIRS += \
./syscfg/ 

C_DEPS += \
./syscfg/clb_config.d 

OBJS += \
./syscfg/clb_config.obj \
./syscfg/clb_sim.obj \
./f2838x_codestartbranch.obj \
./f2838x_usdelay.obj 

ASM_DEPS += \
./f2838x_codestartbranch.d \
./f2838x_usdelay.d 

GEN_MISC_FILES += \
./syscfg/clb_config.h \
./syscfg/clb.dot 

CPP_DEPS += \
./syscfg/clb_sim.d 

GEN_MISC_DIRS__QUOTED += \
"syscfg\" 

OBJS__QUOTED += \
"syscfg\clb_config.obj" \
"syscfg\clb_sim.obj" \
"f2838x_codestartbranch.obj" \
"f2838x_usdelay.obj" 

GEN_MISC_FILES__QUOTED += \
"syscfg\clb_config.h" \
"syscfg\clb.dot" 

C_DEPS__QUOTED += \
"syscfg\clb_config.d" 

CPP_DEPS__QUOTED += \
"syscfg\clb_sim.d" 

GEN_FILES__QUOTED += \
"syscfg\clb_config.c" \
"syscfg\clb_sim.cpp" 

ASM_DEPS__QUOTED += \
"f2838x_codestartbranch.d" \
"f2838x_usdelay.d" 

SYSCFG_SRCS__QUOTED += \
"../MotorDriver.syscfg" 

C_SRCS__QUOTED += \
"./syscfg/clb_config.c" 

CPP_SRCS__QUOTED += \
"./syscfg/clb_sim.cpp" 

ASM_SRCS__QUOTED += \
"../f2838x_codestartbranch.asm" \
"../f2838x_usdelay.asm" 


