TARGET = main
DEBUG = 1
OPT = -Os
CPPSTD =-std=c++17
BUILD_DIR = build

LORA_DEFINES  = -DREGION_EU868
LORA_DEFINES += -DREGION_RU864
LORA_DEFINES += -DUSE_MODEM_LORA
LORA_DEFINES += -DLORA
LORA_DEFINES += -DSX1276MB1LAS
LORA_DEFINES += -DSTM32L152xE
LORA_DEFINES += -DUSE_HAL_DRIVER

######################################
# source
######################################
CPP_SOURCES += src/main.cpp
# LORA_SOURCES += src/lmn_radio.c

ASM_SOURCES = LoRaMac-node/src/boards/NucleoL152/cmsis/arm-gcc/startup_stm32l152xe.s
LDSCRIPT = LoRaMac-node/src/boards/NucleoL152/cmsis/arm-gcc/stm32l152xe_flash.ld

CMSIS_PATH = mculib3/STM32F0_files
INCLUDES += -Isrc
# C_INCLUDES += -I.
# C_INCLUDES += -I$(CMSIS_PATH)
# C_INCLUDES += -I$(CMSIS_PATH)/CMSIS
# INCLUDES += -Imculib3/src
# C_INCLUDES += -Imculib3/src/periph
# C_INCLUDES += -Imculib3/src/bits


LORA_SOURCES += LoRaMac-node/src/radio/sx1276/sx1276.c
LORA_SOURCES += LoRaMac-node/src/system/gpio.c
LORA_SOURCES += LoRaMac-node/src/system/delay.c
LORA_SOURCES += LoRaMac-node/src/system/timer.c
LORA_SOURCES += LoRaMac-node/src/system/fifo.c
LORA_SOURCES += LoRaMac-node/src/system/uart.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/utilities.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/sx1276mb1las-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/cmsis/system_stm32l1xx.c
# LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/spi-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/gpio-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/delay-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/uart-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/rtc-board.c
LORA_SOURCES += LoRaMac-node/src/boards/NucleoL152/lpm-board.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/sysIrqHandlers.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_gpio.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_uart.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_pwr_ex.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc_ex.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cortex.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi_ex.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc.c
LORA_SOURCES += LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc_ex.c

# LORA_SOURCES += LoRaMac-node/src/peripherals/gpio-ioe.c
# LORA_SOURCES += LoRaMac-node/src/peripherals/sx1509.c




LORA_INCLUDES += -ILoRaMac-node/src/boards
LORA_INCLUDES += -ILoRaMac-node/src/system
LORA_INCLUDES += -ILoRaMac-node/src/radio
LORA_INCLUDES += -ILoRaMac-node/src/boards/mcu/stm32/cmsis
LORA_INCLUDES += -ILoRaMac-node/src/boards/NucleoL152/cmsis
LORA_INCLUDES += -ILoRaMac-node/src/boards/NucleoL152
LORA_INCLUDES += -ILoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Inc
LORA_INCLUDES += -ILoRaMac-node/src/mac

# LORA_INCLUDES += -Isrc


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
MCU = -mcpu=cortex-m3 -mthumb

# compile gcc flags
CFLAGS  = $(MCU) $(OPT)
CFLAGS += -Wall -fdata-sections -ffunction-sections -fno-exceptions -fno-strict-volatile-bitfields -fno-threadsafe-statics
CFLAGS += -g -gdwarf-2

CPP_FLAGS += $(CFLAGS) -Wno-register -fno-rtti $(CPPSTD) 

# Generate dependency information
CFLAGS    += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
CPP_FLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# libraries
LIBS = -lc -lm -lnosys

LDFLAGS  = $(MCU) -specs=nano.specs -specs=nosys.specs
LDFLAGS += -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

FLAGS_ = $(CPP_FLAGS) -Og -g -mthumb -g2 -fno-builtin -mcpu=cortex-m3 -Wall -Wextra -pedantic -Wno-unused-parameter -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize
LINKER_FL = -lm -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -mthumb -g2 -mcpu=cortex-m3 -mabi=aapcs -T${LDSCRIPT} -Wl,-Map=${TARGET}.map


# default action: build all
all: submodule clean \
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

LORA_OBJECTS += $(addprefix $(BUILD_DIR)/lora/,$(notdir $(LORA_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(LORA_SOURCES)))

ALL_OBJECTS = $(OBJECTS) $(LORA_OBJECTS) 

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CC) -c $(FLAGS_) -std=gnu99 $(INCLUDES) $(LORA_INCLUDES) $(LORA_DEFINES)  -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

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

clean:
	-rm -fR .dep $(BUILD_DIR)

flash:
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

util:
	st-util

submodule:
	git submodule update --init
	cd mculib3/ && git fetch origin && git checkout dvk
	cd LoRaMac-node/ && git checkout develop

print-%  : ; @echo $* = $($*)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***



	
