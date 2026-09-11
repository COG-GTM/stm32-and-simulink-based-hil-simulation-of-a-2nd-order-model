#!/usr/bin/env bash
# Stage 2 demo — Simulink Embedded Coder generated C: build it, run it, prove it matches the model.
#   ./demo/run_demo.sh            everything (host harness + verification + STM32 cross-compile)
#   SKIP_ARM=1 ./demo/run_demo.sh skip the arm-none-eabi cross-compile
set -euo pipefail
cd "$(dirname "$0")"
OUT=out; mkdir -p "$OUT"
B=$'\033[1;36m'; G=$'\033[1;32m'; Y=$'\033[1;33m'; D=$'\033[0;90m'; N=$'\033[0m'
banner() { printf "\n${B}══ %s ${N}%s\n" "$1" "$(printf '═%.0s' $(seq 1 $((70-${#1}))))"; }

banner "0  Sources: Simulink models -> Embedded Coder (R2021a, ert.tlc, ARM Cortex-M)"
ls -1 ../Simulink/*.slx | sed "s|^|   ${D}|;s|$|${N}|"
for f in ../plant_simulator/SimulinkModels/PlantModel/PlantModel.c \
         ../controller/SimulinkModels/ControllerModel/ControllerModel.c \
         ../controller/SimulinkModels/ReferenceInputModel/ReferenceInputModel.c; do
  printf "   %-64s %s\n" "$f" "$(grep -m1 'generated on' "$f" | sed 's/.*: //')"
done
printf "   plant ODE from PlantModel.c: ${Y}%s${N}\n" "$(grep -o 'TransferFcn_CSTATE\[0\] += -[0-9.]*' ../plant_simulator/SimulinkModels/PlantModel/PlantModel.c | tr '\n' ' ')"
printf "   solver: ${Y}%s${N}  step: ${Y}%s s${N}\n" \
  "$(grep -o '"ode[0-9]"' ../plant_simulator/SimulinkModels/PlantModel/PlantModel.c | tr -d '"')" \
  "$(grep -o 'stepSize0 = [0-9.]*' ../plant_simulator/SimulinkModels/PlantModel/PlantModel.c | sed 's/.*= //')"

banner "1  HOST: compile generated C with gcc (no HAL, no MCU)"
make --no-print-directory -C host_harness -B 2>&1 | grep -v '^mkdir' | sed "s|^|   ${D}|;s|$|${N}|"

banner "2  HOST: execute PlantModel_step() — step, chirp, closed loop (r -> PI -> plant)"
for s in step chirp closedloop; do host_harness/build/harness "$s" "$OUT/$s.csv"; done
printf "   first samples of %s:\n" "$OUT/step.csv"; head -4 "$OUT/step.csv" | sed 's/^/     /'

banner "3  VERIFY: generated code vs analytic Butterworth G(s) (scipy.signal)"
python3 verify.py | tee "$OUT/verify_report.txt"
ls -1 "$OUT"/*.png | sed "s|^|   ${G}plot ${N}|"

if [[ "${SKIP_ARM:-0}" != "1" ]]; then
  banner "4  TARGET: cross-compile the STM32 firmware with arm-none-eabi-gcc"
  if command -v arm-none-eabi-gcc >/dev/null; then
    arm-none-eabi-gcc --version | head -1 | sed 's/^/   /'
    { for t in plant_simulator controller; do
        make --no-print-directory -C firmware TARGET="$t" 2>&1 | grep -v '^  \(CC\|AS\)\|^arm-none-eabi'
      done; } | tee "$OUT/firmware_build.txt"
    ls -la firmware/build/*/*.bin | awk '{printf "   %-52s %6d bytes\n",$9,$5}'
    printf "   ${G}Both images link against the real CubeMX HAL + linker scripts (not flashed: no board attached)${N}\n"
  else
    printf "   ${Y}arm-none-eabi-gcc not found — apt install gcc-arm-none-eabi to build the STM32 images${N}\n"
  fi
fi

banner "DONE"
printf "   artifacts in demo/%s:\n" "$OUT"; ls -1 "$OUT" | sed 's/^/     /'
