################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CFG_SRCS += \
../app.cfg 

CMD_SRCS += \
../EK_TM4C123GXL.cmd 

C_SRCS += \
../E_Paper.c \
../Nokia_5110.c \
../TivaC_GPIO_Config.c \
../main.c 

GEN_CMDS += \
./configPkg/linker.cmd 

GEN_FILES += \
./configPkg/linker.cmd \
./configPkg/compiler.opt 

GEN_MISC_DIRS += \
./configPkg/ 

C_DEPS += \
./E_Paper.d \
./Nokia_5110.d \
./TivaC_GPIO_Config.d \
./main.d 

GEN_OPTS += \
./configPkg/compiler.opt 

OBJS += \
./E_Paper.obj \
./Nokia_5110.obj \
./TivaC_GPIO_Config.obj \
./main.obj 

GEN_MISC_DIRS__QUOTED += \
"configPkg\" 

OBJS__QUOTED += \
"E_Paper.obj" \
"Nokia_5110.obj" \
"TivaC_GPIO_Config.obj" \
"main.obj" 

C_DEPS__QUOTED += \
"E_Paper.d" \
"Nokia_5110.d" \
"TivaC_GPIO_Config.d" \
"main.d" 

GEN_FILES__QUOTED += \
"configPkg\linker.cmd" \
"configPkg\compiler.opt" 

C_SRCS__QUOTED += \
"../E_Paper.c" \
"../Nokia_5110.c" \
"../TivaC_GPIO_Config.c" \
"../main.c" 


