# Cross-compile the STM32 firmware (CubeMX project layout, no STM32CubeIDE needed)
#   make TARGET=plant_simulator      -> STM32F407ZGT6, links PlantModel.c
#   make TARGET=controller           -> STM32F401CDU6, links ControllerModel.c + ReferenceInputModel.c
ROOT    := ../..
TARGET  ?= plant_simulator
BUILD   ?= build/$(TARGET)
PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
SIZE    := $(PREFIX)size
OBJCOPY := $(PREFIX)objcopy

ifeq ($(TARGET),plant_simulator)
  DEVICE  := STM32F407xx
  STARTUP := startup_stm32f407zgtx.s
  LDSCRIPT:= STM32F407ZGTX_FLASH.ld
  MODELS  := SimulinkModels/PlantModel
else
  # CubeMX ships stm32f401xe.h for the F401C(D-E)Ux family
  DEVICE  := STM32F401xE
  STARTUP := startup_stm32f401cdux.s
  LDSCRIPT:= STM32F401CDUX_FLASH.ld
  MODELS  := SimulinkModels/ControllerModel SimulinkModels/ReferenceInputModel
endif

PROJ := $(ROOT)/$(TARGET)
HAL  := $(PROJ)/Drivers/STM32F4xx_HAL_Driver

# Cortex-M4F, hard-float ABI (same as the CubeIDE project settings)
MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  := $(MCU) -D$(DEVICE) -DUSE_HAL_DRIVER -Os -g3 -Wall \
           -ffunction-sections -fdata-sections -fstack-usage -std=gnu11 \
           -I$(PROJ)/Core/Inc -I$(HAL)/Inc -I$(HAL)/Inc/Legacy \
           -I$(PROJ)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
           -I$(PROJ)/Drivers/CMSIS/Include \
           $(addprefix -I$(PROJ)/,$(MODELS))
LDFLAGS := $(MCU) -T$(PROJ)/$(LDSCRIPT) --specs=nosys.specs --specs=nano.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -lc -lm -lnosys

SRCS := $(wildcard $(PROJ)/Core/Src/*.c) \
        $(wildcard $(HAL)/Src/*.c) \
        $(foreach m,$(MODELS),$(wildcard $(PROJ)/$(m)/*.c))
SRCS := $(filter-out %_template.c,$(SRCS))
OBJS := $(patsubst $(ROOT)/%.c,$(BUILD)/%.o,$(SRCS)) $(BUILD)/$(TARGET)/Core/Startup/$(STARTUP:.s=.o)

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin
	@echo
	@echo "== $(TARGET) ($(DEVICE)) =="
	@$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.o: $(ROOT)/%.c
	@mkdir -p $(dir $@)
	@$(CC) $(CFLAGS) -c $< -o $@ && echo "  CC  $(notdir $<)"

$(BUILD)/$(TARGET)/Core/Startup/%.o: $(PROJ)/Core/Startup/%.s
	@mkdir -p $(dir $@)
	@$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@ && echo "  AS  $(notdir $<)"

clean:
	rm -rf build

.PHONY: all clean
