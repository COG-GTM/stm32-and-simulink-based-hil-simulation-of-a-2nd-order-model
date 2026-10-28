# Host SIL regression gate

Compiles the Embedded Coder generated `PlantModel.c`, `ControllerModel.c` and
`ReferenceInputModel.c` unmodified with host gcc, runs them in the same order as
the two STM32 timer callbacks (lock-stepped at `sample_time`), and checks the
result against the Simulink model. Runs in CI on every push/PR.

```sh
sudo apt-get install octave && pip install -r sil/requirements.txt
make -C sil            # exit 1 on any regression; report in sil/out/sil_report.html
```

Oracle (no MATLAB needed):
- `SystemParameters.m` / `SampleTime.m` evaluated by Octave (`export_params.m`).
- PI gains, integrator method, reference pulse/constant read from the `.slx` XML.

Checks:
- Traceability: constants in the generated C equal the model/parameter values.
- Open loop: generated plant vs analytic `G(s)` step response.
- Closed loop SIL vs MIL (exact ZOH plant + trapezoidal PI, float64): max error,
  rise / settling time, overshoot, actuator within PWM range.

Override sources to test a candidate regeneration, e.g.
`make -C sil CTRL_DIR=/path/to/new/ControllerModel OUT=out-new`.
