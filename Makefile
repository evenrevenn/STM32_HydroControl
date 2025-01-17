##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.19.2] date: [Fri Mar 01 22:59:04 SAMT 2024]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = RTOS_example


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT_C = -Og
OPT_CXX = -Og -fno-rtti -fno-use-cxa-atexit -ffreestanding


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
FreeRTOS_core/croutine.c \
FreeRTOS_core/event_groups.c \
FreeRTOS_core/heap_4.c \
FreeRTOS_core/list.c \
FreeRTOS_core/port.c \
FreeRTOS_core/queue.c \
FreeRTOS_core/tasks.c \
FreeRTOS_core/timers.c
# Drivers/ST_Library/src/stm32f10x_can.c \
# Drivers/ST_Library/src/stm32f10x_gpio.c \
# Drivers/ST_Library/src/stm32f10x_i2c.c \
# Drivers/ST_Library/src/stm32f10x_lib.c \
# Drivers/ST_Library/src/stm32f10x_nvic.c \
# Drivers/ST_Library/src/stm32f10x_rcc.c \
# Drivers/ST_Library/src/stm32f10x_spi.c \
# Drivers/ST_Library/src/stm32f10x_systick.c \
# Drivers/ST_Library/src/stm32f10x_usart.c \

# CPP sources
CXX_SOURCES =  \
Core/Src/main.cpp \
Core/Src/UART_handler.cpp \
Core/Src/circular_buffer.cpp \
Core/Src/angle_detectors.cpp \
Core/Src/motors_handler.cpp \
Core/Src/i2c_slave.cpp \
Core/Src/i2c_master.cpp \
Core/Src/motion_control.cpp


# ASM sources
ASM_SOURCES =  \
startup_stm32f103xb.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)g++ -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)g++ -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32F103xB


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/Core/Include \
-IDrivers/CMSIS/DSP/Include \
-IFreeRTOS_core/include \
-IFreeRTOS_core
# -IDrivers/ST_Library/inc \

# CPP includes
CXX_INCLUDES =  \
$(C_INCLUDES) 
# "c:/program files (x86)/gnu arm embedded toolchain/10 2021.10/arm-none-eabi/include"

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT_C) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT_C) -Wall -fdata-sections -ffunction-sections

CXXFLAGS = $(MCU) $(C_DEFS) $(CXX_INCLUDES) $(OPT_CXX) -Wall -fdata-sections -ffunction-sections

CXXFLAGS += -std=c++20 -Wno-volatile

ifeq ($(DEBUG), 1)
CFLAGS += -g3 -gdwarf-2
endif

ifeq ($(DEBUG), 1)
CXXFLAGS += -g3 -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Generate dependency information
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103C8Tx_FLASH.ld

# libraries
LIBS = -lc_nano -lstdc++_nano -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
# LDFLAGS = $(MCU) -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

prog: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify exit reset"

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rd /s /q $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
