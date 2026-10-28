# Host SIL regression gate for the Embedded Coder generated C.
#   make -C sil          build, run, verify against the model (exit 1 on regression)
CC        ?= gcc
CFLAGS    ?= -std=c99 -O2 -Wall -Werror
PYTHON    ?= python3
OCTAVE    ?= octave
OUT       ?= out
PLANT_DIR ?= ../plant_simulator/SimulinkModels/PlantModel
CTRL_DIR  ?= ../controller/SimulinkModels/ControllerModel
REF_DIR   ?= ../controller/SimulinkModels/ReferenceInputModel

HDRS = $(wildcard $(PLANT_DIR)/*.h $(CTRL_DIR)/*.h $(REF_DIR)/*.h)
SLX  = $(wildcard ../Simulink/PlantModel.slx ../Simulink/ControllerModel.slx ../Simulink/ReferenceInputModel.slx)
SRCS = sil_harness.c $(PLANT_DIR)/PlantModel.c $(CTRL_DIR)/ControllerModel.c \
       $(REF_DIR)/ReferenceInputModel.c

.PHONY: sil clean
.DELETE_ON_ERROR:
sil: $(OUT)/results.json

$(OUT)/sil_harness: $(SRCS) $(HDRS) | $(OUT)
	$(CC) $(CFLAGS) -I$(PLANT_DIR) -I$(CTRL_DIR) -I$(REF_DIR) $(SRCS) -lm -o $@

$(OUT)/params.json: export_params.m ../Simulink/SystemParameters.m ../Simulink/SampleTime.m | $(OUT)
	$(OCTAVE) --no-gui --quiet --no-window-system export_params.m > $@

$(OUT)/results.json: $(OUT)/sil_harness $(OUT)/params.json verify.py $(SLX)
	$(OUT)/sil_harness open 160 1.0 > $(OUT)/open_loop.csv
	$(OUT)/sil_harness closed 320 > $(OUT)/closed_loop.csv
	$(PYTHON) verify.py --out $(OUT) --plant-c $(PLANT_DIR)/PlantModel.c \
	  --ctrl-c $(CTRL_DIR)/ControllerModel.c --ref-c $(REF_DIR)/ReferenceInputModel.c

$(OUT):
	mkdir -p $@

clean:
	rm -rf $(OUT)
