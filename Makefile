TARGET_F0 = main
DEBUG = 1
OPT = -Os
CPPSTD =-std=c++17
BUILD_DIR = build

LORA_DEFINES  = -DREGION_RU864
LORA_DEFINES += -DUSE_MODEM_LORA
LORA_DEFINES += -DSX1276MB1LAS
LORA_DEFINES += -DSTM32L152xE
LORA_DEFINES += -DUSE_HAL_DRIVER

######################################
# source
######################################
CPP_SOURCES_F0 = src/main.cpp

ASM_SOURCES_F0 = LoRaMac-node/src/boards/NucleoL152/cmsis/arm-gcc/startup_stm32l152xe.s
LDSCRIPT_F0 = LoRaMac-node/src/boards/NucleoL152/cmsis/arm-gcc/stm32l152xe_flash.ld

CMSIS_PATH = mculib3/STM32F0_files
C_INCLUDES =  
# C_INCLUDES += -I.
# C_INCLUDES += -I$(CMSIS_PATH)
# C_INCLUDES += -I$(CMSIS_PATH)/CMSIS
# C_INCLUDES += -Imculib3/src
# C_INCLUDES += -Imculib3/src/periph
# C_INCLUDES += -Imculib3/src/bits

LORA_SOURCES += LoRaMac-node/src/radio/sx1276/sx1276.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/sx1276mb1las-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/cmsis/system_stm32l1xx.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/board.c
LORA_SOURCES += LoRaMac-node/src/system/gpio.c
LORA_SOURCES += LoRaMac-node/src/system/delay.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/spi-board.c
LORA_SOURCES += LoRaMac-node/src/system/timer.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/utilities.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/rtc-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/lpm-board.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr_ex.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc_ex.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cortex.c
LORA_SOURCES += LoRaMac-node/src/system/fifo.c
LORA_SOURCES += LoRaMac-node/src/system/uart.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/gpio-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/delay-board.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc_ex.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/uart-board.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_gpio.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_uart.c




LORA_INCLUDES += -ILoRaMac-node/src/boards
LORA_INCLUDES += -ILoRaMac-node/src/system
LORA_INCLUDES += -ILoRaMac-node/src/radio
LORA_INCLUDES += -ILoRaMac-node/src/boards/mcu/stm32/cmsis
LORA_INCLUDES += -ILoRaMac-node/src/boards/NucleoL152/cmsis
LORA_INCLUDES += -ILoRaMac-node/src/boards/NucleoL152
LORA_INCLUDES += -ILoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Inc


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
CPU_F0 = -mcpu=cortex-m3

# NONE for Cortex-M0/M0+/M3
FPU_F0 =

FLOAT-ABI_F0 =

# mcu
MCU_F0 = $(CPU_F0) -mthumb $(FPU_F0) $(FLOAT-ABI_F0)

# compile gcc flags
ASFLAGS_F0 = $(MCU_F0) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS_F0  = $(MCU_F0) $(C_DEFS_F0) $(C_INCLUDES) $(C_INCLUDES_F0) $(LORA_INCLUDES) $(OPT)
CFLAGS_F0 += -Wall -fdata-sections -ffunction-sections -fno-exceptions -fno-strict-volatile-bitfields
CFLAGS_F0 += -g -gdwarf-2

CPP_FLAGS += -Wno-register -fno-rtti $(CPPSTD)


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
all: submodule clean \
$(BUILD_DIR)/$(TARGET_F0).elf $(BUILD_DIR)/$(TARGET_F0).hex $(BUILD_DIR)/$(TARGET_F0).bin 

	


#######################################
# build the application
#######################################
# list of objects
OBJECTS_F0 += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES_F0:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES_F0)))
# OBJECTS_F0 += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES_F0:.c=.o)))
# vpath %.c $(sort $(dir $(CPP_SOURCES_F0)))
OBJECTS_F0 += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES_F0:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES_F0)))

LORA_OBJECTS += $(addprefix $(BUILD_DIR)/lora/,$(notdir $(LORA_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(LORA_SOURCES)))

ALL_OBJECTS = $(OBJECTS_F0) $(LORA_OBJECTS) 

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS_F0) $(LORA_DEFINES) $(CPP_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

# $(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
# 	$(CC) -c $(CFLAGS_F0) $(CPPSTD) $(LORA_DEFINES) -fno-rtti -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR) 
	$(AS) -c $(CFLAGS_F0) $< -o $@

$(BUILD_DIR)/lora/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS_F0) $(LORA_DEFINES) -Wa,-a,-ad,-alms=$(BUILD_DIR)/lora/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/$(TARGET_F0).elf: $(ALL_OBJECTS) Makefile
	$(CC) $(ALL_OBJECTS) $(LDFLAGS_F0) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@
	mkdir $@/lora

clean:
	-rm -fR .dep $(BUILD_DIR)

flash:
	st-flash write $(BUILD_DIR)/$(TARGET_F0).bin 0x8000000

util:
	st-util

submodule:
	git submodule update --init
	cd mculib3/ && git checkout develop
	cd LoRaMac-node/ && git checkout develop

print-%  : ; @echo $* = $($*)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***



	
	