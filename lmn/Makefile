TARGET = lib_lmn
DEBUG = 1
OPT = -Os
CPPSTD =-std=c++17
BUILD_DIR = build


LORA_DEFINES += -D$(RADIO)
LORA_DEFINES += -D$(MCU)
LORA_DEFINES += -DUSE_HAL_DRIVER

######################################
# source
######################################
CPP_SOURCES += src/lmn_radio.cpp

ASM_SOURCES = ../LoRaMac-node/src/boards/NucleoL152/cmsis/arm-gcc/startup_stm32l152xe.s
LDSCRIPT = ../LoRaMac-node/src/boards/NucleoL152/cmsis/arm-gcc/stm32l152xe_flash.ld

LORA_SOURCES += ../LoRaMac-node/src/radio/sx1276/sx1276.c
LORA_SOURCES += ../LoRaMac-node/src/system/gpio.c
LORA_SOURCES += ../LoRaMac-node/src/system/delay.c
LORA_SOURCES += ../LoRaMac-node/src/system/timer.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/utilities.c
LORA_SOURCES += ../LoRaMac-node/src/boards/NucleoL152/sx1276mb1las-board.c
LORA_SOURCES += ../LoRaMac-node/src/boards/NucleoL152/cmsis/system_stm32l1xx.c
LORA_SOURCES += ../LoRaMac-node/src/boards/NucleoL152/spi-board.c
LORA_SOURCES += ../LoRaMac-node/src/boards/NucleoL152/gpio-board.c
LORA_SOURCES += ../LoRaMac-node/src/boards/NucleoL152/delay-board.c
LORA_SOURCES += ../LoRaMac-node/src/boards/NucleoL152/rtc-board.c
LORA_SOURCES += ../LoRaMac-node/src/boards/NucleoL152/lpm-board.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_gpio.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rcc_ex.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_cortex.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_spi_ex.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc.c
LORA_SOURCES += ../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Src/stm32l1xx_hal_rtc_ex.c





LORA_INCLUDES += -I../LoRaMac-node/src/boards
LORA_INCLUDES += -I../LoRaMac-node/src/system
LORA_INCLUDES += -I../LoRaMac-node/src/radio
LORA_INCLUDES += -I../LoRaMac-node/src/boards/mcu/stm32/cmsis
LORA_INCLUDES += -I../LoRaMac-node/src/boards/NucleoL152/cmsis
LORA_INCLUDES += -I../LoRaMac-node/src/boards/NucleoL152
LORA_INCLUDES += -I../LoRaMac-node/src/boards/mcu/stm32/STM32L1xx_HAL_Driver/Inc
LORA_INCLUDES += -I../LoRaMac-node/src/mac


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-

CPP = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
LD = $(PREFIX)ld
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# compile gcc flags
CFLAGS  = -mcpu=cortex-m3 -mthumb $(OPT)
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
FLAGS_ = -Og -g -mthumb -g2 -fno-builtin -mcpu=cortex-m3 -Wall -Wextra -pedantic -Wno-unused-parameter -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize

# default action: build all
all: clean $(BUILD_DIR)/$(TARGET).o

	


#######################################
# build the application
#######################################
# list of objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
LORA_OBJECTS += $(addprefix $(BUILD_DIR)/lora/,$(notdir $(LORA_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(LORA_SOURCES)))

ALL_OBJECTS = $(OBJECTS) $(LORA_OBJECTS) 

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CC) -c $(FLAGS_) $(CPP_FLAGS) $(INCLUDES) $(LORA_INCLUDES) $(LORA_DEFINES)  -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR) 
	$(AS) -c $(FLAGS_) -x assembler-with-cpp $< -o $@

$(BUILD_DIR)/lora/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(FLAGS_) -std=gnu99 $(LORA_INCLUDES) $(LORA_DEFINES) -Wa,-a,-ad,-alms=$(BUILD_DIR)/lora/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/$(TARGET).o: $(ALL_OBJECTS) Makefile
	$(LD) -r $(ALL_OBJECTS) -o $@

	
$(BUILD_DIR):
	mkdir $@
	mkdir $@/lora

clean:
	-rm -fR .dep $(BUILD_DIR)

print-%  : ; @echo $* = $($*)
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***



	
