TARGET_F0 = main
DEBUG = 1
OPT = -Os
CPPSTD =-std=c++17
BUILD_DIR = build

######################################
# source
######################################
CPP_SOURCES_F0 = main.cpp
CPP_SOURCES_IN = in/file.cpp
CMSIS_PATH = mculib3/STM32F0_files

ASM_SOURCES_F0 = $(CMSIS_PATH)/startup_stm32f030x6.s
LDSCRIPT_F0 = $(CMSIS_PATH)/STM32F030K6Tx_FLASH.ld

# C includes
C_INCLUDES =  
C_INCLUDES += -I.
C_INCLUDES += -I$(CMSIS_PATH)
C_INCLUDES += -I$(CMSIS_PATH)/CMSIS
C_INCLUDES += -Imculib3/src
C_INCLUDES += -Imculib3/src/periph
C_INCLUDES += -Imculib3/src/bits
C_INCLUDES += -Iin
C_INCLUDES_IN = -Iin


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-

CPP = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
CPU_F0 = -mcpu=cortex-m0

# NONE for Cortex-M0/M0+/M3
FPU_F0 =

FLOAT-ABI_F0 =

# mcu
MCU_F0 = $(CPU_F0) -mthumb $(FPU_F0) $(FLOAT-ABI_F0)

# compile gcc flags
ASFLAGS_F0 = $(MCU_F0) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS_F0  = $(MCU_F0) $(C_DEFS_F0) $(C_INCLUDES) $(C_INCLUDES_F0) $(OPT)
CFLAGS_F0 += -Wall -Wno-register -fdata-sections -ffunction-sections -fno-exceptions -fno-strict-volatile-bitfields
CFLAGS_F0 += -g -gdwarf-2 


# Generate dependency information
CFLAGS_F0 += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# libraries
LIBS = -lc -lm -lnosys

LDFLAGS_F0  = $(MCU_F0) -specs=nano.specs -specs=nosys.specs
LDFLAGS_F0 += -T$(LDSCRIPT_F0) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET_F0).map,--cref -Wl,--gc-sections

# default action: build all
all: clean \
$(BUILD_DIR)/$(TARGET_F0).elf $(BUILD_DIR)/$(TARGET_F0).hex $(BUILD_DIR)/$(TARGET_F0).bin
	


#######################################
# build the application
#######################################
# list of objects
OBJECTS_F0 += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES_F0:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES_F0)))
OBJECTS_F0 += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES_IN:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES_IN)))
OBJECTS_F0 += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES_F0:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES_F0)))



$(BUILD_DIR)/$(TARGET_F0).o:$(CPP_SOURCES_F0) Makefile | $(BUILD_DIR) 
	$(CPP) -c $(CFLAGS_F0) $(CPPSTD) -fno-rtti -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/file.o:$(CPP_SOURCES_IN) Makefile | $(BUILD_DIR) 
	$(CPP) -c $(CFLAGS_F0) $(CPPSTD) -DSTM32F030x6 -DF_CPU=48000000UL -flto -fno-rtti -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/startup_stm32f030x6.o: $(ASM_SOURCES_F0) Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS_F0) $< -o $@

$(BUILD_DIR)/$(TARGET_F0).elf: $(OBJECTS_F0) Makefile
	$(CPP) $(OBJECTS_F0) $(LDFLAGS_F0) -flto -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@

clean:
	-rm -fR .dep $(BUILD_DIR)

flash:
	st-flash write $(BUILD_DIR)/$(TARGET_F0).bin 0x8000000

util:
	st-util

update:
	git submodule update --remote mculib3
	cd mculib3/ && git checkout develop
	git submodule update --remote LoRaMac-node
	cd LoRaMac-node/ && git checkout develop

test_:
	$(MAKE) -C ./test/
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***



	
	