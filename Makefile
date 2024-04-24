##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.18.0-B7] date: [Fri Jan 13 16:50:57 BRT 2023]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

MAKEFLAGS = -j$(nproc)
# MAKEFLAGS = -j$(grep -c ^processor /proc/cpuinfo)
######################################
# target
######################################
TARGET = stm32fx


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
# Os - Performs optimizations to reduce the code size at the expense of a possible increase in execution time. This option aims for a balanced code size reduction and fast performance.
# Oz - Optimizes for smaller code size.
OPT = -Oz

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

SRC_DIR = src
DRIVER_DIR = Drivers/STM32F1xx_HAL_Driver/Src
INC_DIR = src/include

######################################
# source
######################################
# C sources
# $(SRC_DIR)/usart.c

C_SOURCES =  \
$(SRC_DIR)/system_main.c \
$(SRC_DIR)/adc.c \
$(SRC_DIR)/i2c.c \
$(SRC_DIR)/iwdg.c \
$(SRC_DIR)/tim.c \
$(SRC_DIR)/stm32_log.c \
$(SRC_DIR)/stm32f1xx_hal_msp.c \
$(SRC_DIR)/stm32f1xx_it.c \
$(SRC_DIR)/system_stm32f1xx.c \
$(DRIVER_DIR)/stm32f1xx_hal_adc.c \
$(DRIVER_DIR)/stm32f1xx_hal_i2c.c \
$(DRIVER_DIR)/stm32f1xx_hal.c \
$(DRIVER_DIR)/stm32f1xx_hal_rcc.c \
$(DRIVER_DIR)/stm32f1xx_hal_rcc_ex.c \
$(DRIVER_DIR)/stm32f1xx_hal_gpio.c \
$(DRIVER_DIR)/stm32f1xx_hal_cortex.c \
$(DRIVER_DIR)/stm32f1xx_hal_pwr.c \
$(DRIVER_DIR)/stm32f1xx_hal_exti.c \
$(DRIVER_DIR)/stm32f1xx_hal_tim.c \
$(DRIVER_DIR)/stm32f1xx_hal_iwdg.c \
$(DRIVER_DIR)/stm32f1xx_hal_tim_ex.c \
$(DRIVER_DIR)/stm32f1xx_hal_dma.c

# $(DRIVER_DIR)/stm32f1xx_hal_gpio_ex.c \
# $(DRIVER_DIR)/stm32f1xx_hal_flash.c \
# $(DRIVER_DIR)/stm32f1xx_hal_flash_ex.c \
# $(DRIVER_DIR)/stm32f1xx_hal_uart.c \


C_SOURCES2=$(shell find -L $(SRC_DIR) -name '*.c')
C_SOURCES2+=$(shell find -L $(DRIVER_DIR) -name '*.c')

CXX_SOURCES2=$(shell find -L $(SRC_DIR) -name '*.cpp')


# CPP sources
CXX_SOURCES = \
$(SRC_DIR)/main.cpp \
$(SRC_DIR)/test_functions.cpp \
components/helper/delay.cpp \
components/math/dsp.cpp \
components/modules/pcy8575/pcy8575.cpp \
components/peripherals/adc/adc_driver.cpp \
components/modules/aht10/aht10.cpp \
components/modules/ks0066/ks0066.cpp \
components/modules/lcd/lcd.cpp \
components/modules/hx711/hx711.cpp \
components/peripherals/gpio/gpio_driver.cpp \
components/peripherals/backup/backup.cpp \
components/peripherals/backup/reset_reason.cpp \
components/peripherals/i2c/i2c_driver.cpp \
components/peripherals/tim/tim_driver.cpp \
components/systems/weighing_controller/weighing_scale.cpp \
components/systems/weighing_controller/load_cell/load_cell.cpp

# $(SRC_DIR)/usart.cpp

# ASM sources
ASM_SOURCES =  \
startup_stm32f103xb.s

#######################################
# binaries
#######################################
# COMPILER_PATH = ~/st/arm-gnu-toolchain-12.2.mpacbti-bet1-x86_64-arm-none-eabi/bin/
# PREFIX = $(COMPILER_PATH)arm-none-eabi-
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
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

# add this flag to print float: -u _printf_float
FLOAT-PRINT = -u _printf_float

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI) $(FLOAT-PRINT) -specs=nosys.specs -specs=nano.specs
# MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI) -specs=nosys.specs -specs=nano.specs

# MCU = $(CPU) -mthumb -fmessage-length=0 -fsigned-char -Wall -Wextra -ffunction-sections -fdata-sections#$(FPU) $(FLOAT-ABI)
# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DDEBUG \
-DTRACE \
-DSTM32F103xB \
-DUSE_HAL_DRIVER
# -DOS_USE_TRACE_SEMIHOSTING_DEBUG
# -DHSE_VALUE=8000000
# -DUSE_FULL_ASSERT
# AS includes

AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-I$(INC_DIR) \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include

# CPP includes
CXX_INCLUDES = \
-I$(INC_DIR) \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include \
-Icomponents/helper/include \
-Icomponents/math/include \
-Icomponents/modules/aht10/include \
-Icomponents/modules/hx711/include \
-Icomponents/modules/ks0066/include \
-Icomponents/modules/lcd/include \
-Icomponents/modules/pcy8575/include \
-Icomponents/peripherals/adc/include \
-Icomponents/peripherals/backup/include \
-Icomponents/peripherals/gpio/include \
-Icomponents/peripherals/i2c/include \
-Icomponents/peripherals/tim/include \
-Icomponents/systems/weighing_controller/include \
-Icomponents/systems/weighing_controller/load_cell/include


# compile gcc flags
#-fno-exceptions: C++ only. Disables the generation of code that is needed to support C++ exceptions.
#-fno-rtti: C++ only. Disables the generation of code that is needed to support Run Time Type Information (RTTI) features.
#-flto: Enables Link Time Optimization (LTO), which enables the linker to make additional optimizations across multiple source files.
# Reference: https://developer.arm.com/documentation/100748/0612/writing-optimized-code/optimizing-for-code-size-or-performance
# Reference: https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
C_EXTRAFLAGS = -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra -g3 -flto
CXX_EXTRAFLAGS = -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra -g3 -flto

C_EXTRAFLAGS2 = -std=c99 -Wno-unused-parameter -Wno-conversion -Wno-sign-conversion -Wno-bad-function-cast -Wno-unused-variable -Wno-implicit-function-declaration #-std=gnu11
CXX_EXTRAFLAGS2 = -std=c++17 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics #-std=gnu++11

ASFLAGS = $(MCU) $(OPT) $(AS_DEFS) $(AS_INCLUDES)
CFLAGS += $(MCU) $(OPT) $(C_EXTRAFLAGS) $(C_DEFS) $(C_INCLUDES) $(C_EXTRAFLAGS2)

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# CPP Flags
# CXXFLAGS = $(CXX_INCLUDES) -std=c++17 -Wno-register

CXXFLAGS = $(MCU) $(OPT) $(CXX_EXTRAFLAGS) $(C_DEFS) $(CXX_INCLUDES) $(CXX_EXTRAFLAGS2)
# CXXFLAGS = $(MCU) $(C_DEFS) $(CXX_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CXXFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
#######################################


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103C8Tx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys #-lstdc++ #-lgcc #-fno-common#-lrdimon #--data-sections
LIBDIR = 
LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections --lto_level

# -T mem.ld -T libs.ld -T sections.ld -nostartfiles -Xlinker --gc-sections -L"../ldscripts"
LDFLAGS2 = $(MCU) $(OPT) $(C_EXTRAFLAGS) -T$(LDSCRIPT) $(LIBDIR) -I$(CXX_INCLUDES)-Wl,-Map=$(BUILD_DIR)/$(TARGET).map

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of CPP objects
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
	$(CXX) $(LDFLAGS2) $(OBJECTS) -o $@
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
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

flash: all
	st-flash --reset --format ihex write $(BUILD_DIR)/$(TARGET).hex
# --clock=48		, means 8 MHz of main clock source
# --trace=20000		,  -tXX, --trace=XX      Specify the trace frequency in Hz
#
monitor: flash
	st-trace --clock=72
monitor2:
	st-trace --clock=72

size:
	$(SZ) $(BUILD_DIR)/$(TARGET).hex
	$(SZ) $(BUILD_DIR)/$(TARGET).elf

erase:
	st-flash erase

flash_openocd: all
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify exit reset"
# *** EOF ***
