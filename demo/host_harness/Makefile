#  Host build of the Embedded Coder generated code (no HAL, no STM32 — plain gcc)
ROOT      := ../..
PLANT     := $(ROOT)/plant_simulator/SimulinkModels/PlantModel
CTRL      := $(ROOT)/controller/SimulinkModels/ControllerModel
REF       := $(ROOT)/controller/SimulinkModels/ReferenceInputModel
BUILD     ?= build

CC        ?= gcc
CFLAGS    ?= -O2 -std=c99 -Wall -Wno-unused-function
CPPFLAGS  := -I$(PLANT) -I$(CTRL) -I$(REF)
LDLIBS    := -lm

SRCS := main.c $(PLANT)/PlantModel.c $(CTRL)/ControllerModel.c $(REF)/ReferenceInputModel.c

$(BUILD)/harness: $(SRCS) | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)

.PHONY: clean
