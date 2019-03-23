TARGET = main
DEBUG = 1
OPT = -Os
CPPSTD =-std=c++17
BUILD_DIR = build

RADIO = SX1276MB1LAS
MCU   = STM32L152xE

######################################
# source
######################################
CPP_SOURCES += main.cpp
LIB_OBJECTS += lmn/build/lib_lmn.o

# ASM_SOURCES = LoRaMac-node/src/boards/NucleoL152/cmsis/arm-gcc/startup_stm32l152xe.s
LDSCRIPT = LoRaMac-node/src/boards/NucleoL152/cmsis/arm-gcc/stm32l152xe_flash.ld

INCLUDES += -Ilmn


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
# mcu
MCU_ = -mcpu=cortex-m3 -mthumb

# compile gcc flags
CFLAGS  = $(MCU_) $(OPT)
CFLAGS += -Wall -fdata-sections -ffunction-sections -fno-exceptions -fno-strict-volatile-bitfields 
CFLAGS += -g -gdwarf-2

CPP_FLAGS += $(CFLAGS) -Wno-register -fno-rtti $(CPPSTD) -fno-threadsafe-statics

# Generate dependency information
CFLAGS    += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
CPP_FLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# libraries
LIBS = -lc -lm -lnosys

LDFLAGS  = $(MCU_) -specs=nano.specs -specs=nosys.specs
LDFLAGS += -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

FLAGS_ = -Og -g -mthumb -g2 -fno-builtin -mcpu=cortex-m3 -Wall -Wextra -pedantic -Wno-unused-parameter -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize
LINKER_FL = -lm -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -mthumb -g2 -mcpu=cortex-m3 -mabi=aapcs -T${LDSCRIPT} -Wl,-Map=$(BUILD_DIR)/${TARGET}.map


# default action: build all
all: submodule clean lora \
$(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin 

	


#######################################
# build the application
#######################################
# list of objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.c=.o)))
# vpath %.c $(sort $(dir $(CPP_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

ALL_OBJECTS = $(OBJECTS) $(LIB_OBJECTS)

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CC) -c $(FLAGS_) $(CPP_FLAGS) $(INCLUDES) $(LORA_INCLUDES) $(LORA_DEFINES)  -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

# $(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
# 	$(CC) -c $(CFLAGS) $(CPPSTD) $(LORA_DEFINES) -fno-rtti -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR) 
	$(AS) -c $(FLAGS_) -x assembler-with-cpp $< -o $@

$(BUILD_DIR)/lora/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(FLAGS_) -std=gnu99 $(LORA_INCLUDES) $(LORA_DEFINES) -Wa,-a,-ad,-alms=$(BUILD_DIR)/lora/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(ALL_OBJECTS) Makefile
	$(CC) $(ALL_OBJECTS) $(LINKER_FL) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@
	mkdir $@/lora

lora:
	cd lmn && make RADIO=$(RADIO) MCU=$(MCU)

clean:
	-rm -fR .dep $(BUILD_DIR)

flash:
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

util:
	st-util

submodule:
	git submodule update --init
	cd mculib3/ && git fetch && git checkout v1.00
	cd LoRaMac-node/ && git checkout develop

print-%  : ; @echo $* = $($*)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***



	
