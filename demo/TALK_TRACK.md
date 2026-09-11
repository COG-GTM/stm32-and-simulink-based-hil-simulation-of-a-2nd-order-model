# Stage 2 — Auto-code generation (Simulink Embedded Coder)

## Talk track (~140 words)

Stage one gave us the plant and controller as Simulink models. Nobody hand-writes C here:
Embedded Coder turns `PlantModel.slx` into `PlantModel.c` — an ode4 Runge-Kutta integrator
of the 2nd-order Butterworth filter, targeted at ARM Cortex-M, 50 ms fixed step.

The question every safety reviewer asks is: does the generated code equal the model?
We answer it before touching silicon. The same generated C is compiled with plain gcc on
the host, stepped with a unit step and a 0.05–3 Hz chirp, and overlaid on the analytic
transfer function from scipy. Peak deviation: tens of microvolts on the step — solver
truncation, not a code error. The frequency envelope lands on the Bode curve at 0.8 Hz.

Then we link three generated models together — reference, PI controller, plant — and get
the closed loop on the desktop. Finally, the identical sources cross-compile with
arm-none-eabi-gcc into the STM32F407 and F401 images: 16 KB and 11 KB of flash.

## What you're seeing

- `run_demo.sh` terminal: generated-code provenance (R2021a, ert.tlc, ode4, 0.05 s), gcc build, 321-step execution, verdict `PASS`, two STM32 images linked.
- `01_step_chirp_vs_analytic.png`: Embedded Coder C (red dashed) on top of `scipy.signal.lsim` (black); error panel in the 1e-5…1e-4 V range.
- `02_frequency_response.png`: chirp-envelope gain of the generated code sitting on the analytic Bode magnitude, corner at f_n = 0.796 Hz.
- `03_closed_loop.png`: `ReferenceInputModel.c → ControllerModel.c → PlantModel.c` running together on host — zero steady-state error.
- `firmware_build.txt`: `arm-none-eabi-size` of both real firmware images, built from the CubeMX HAL + linker scripts in this repo.

## Hand-off

Next: Stage 3 flashes this generated plant onto the STM32 and closes the loop in real time with a bare-metal PID controller over PWM/ADC — hardware-in-the-loop.
