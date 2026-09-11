/*
 * Host-side harness for the Simulink Embedded Coder generated models.
 *
 * Links the UNMODIFIED generated sources:
 *   plant_simulator/SimulinkModels/PlantModel/PlantModel.c        (ode4, Ts = 50 ms)
 *   controller/SimulinkModels/ControllerModel/ControllerModel.c   (discrete PI, Ts = 50 ms)
 *   controller/SimulinkModels/ReferenceInputModel/ReferenceInputModel.c (1.0/1.5 V pulse ref)
 *
 * On the STM32F407 the plant step is driven from HAL_TIM_PeriodElapsedCallback
 * (TIM2 @ 40 Hz, divided by 2 -> 20 Hz).  Here the same PlantModel_step() is
 * called from a plain loop and the I/O is written to CSV instead of DAC/USART.
 *
 * Usage: harness <scenario> <out.csv>     scenario = step | chirp | closedloop
 */
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PlantModel.h"
#include "ControllerModel.h"
#include "ReferenceInputModel.h"

#define TS          0.05    /* Simulink fixed-step sample time (SampleTime.m)   */
#define T_END       16.0    /* seconds simulated (2 reference periods)          */
#define CHIRP_F0    0.05    /* Hz  – chirp start                                */
#define CHIRP_F1    3.0     /* Hz  – chirp end                                  */
#define VOLT_REF    3.3     /* DAC/PWM full scale on the target                 */

static double chirp(double t)
{
  /* linear chirp, same formula used by scipy.signal.chirp(method='linear') */
  double k = (CHIRP_F1 - CHIRP_F0) / T_END;
  return 1.0 * sin(2.0 * M_PI * (CHIRP_F0 * t + 0.5 * k * t * t));
}

int main(int argc, char **argv)
{
  if (argc != 3) {
    fprintf(stderr, "usage: %s step|chirp|closedloop out.csv\n", argv[0]);
    return 2;
  }
  const char *scenario = argv[1];
  FILE *f = fopen(argv[2], "w");
  if (!f) { perror(argv[2]); return 1; }

  PlantModel_initialize();
  ControllerModel_initialize();
  ReferenceInputModel_initialize();

  const int n = (int)(T_END / TS) + 1;
  int closed = strcmp(scenario, "closedloop") == 0;

  /* y      : PlantModel_Y.y as read on the STM32 right after PlantModel_step()
   *          (ERT computes the outport before the ode4 update, then the minor
   *          steps overwrite it with the RK4 stage-4 predictor for t+Ts)
   * y_rk4  : 25*x2 = plant output rebuilt from the integrated continuous state
   *          after the step, i.e. the exact ode4 estimate of y(t+Ts)          */
  fprintf(f, closed ? "t,r,e,u,y,y_rk4\n" : "t,u,y,y_rk4\n");

  double y_prev = 0.0;                         /* plant output seen by controller */
  for (int k = 0; k < n; k++) {
    double t = k * TS;
    double u, r = 0.0, e = 0.0;

    if (closed) {
      ReferenceInputModel_step();              /* generated reference (1.0/1.5 V) */
      r = ReferenceInputModel_Y.r;
      e = r - y_prev;
      ControllerModel_U.e = (real_T)e;
      ControllerModel_step();                  /* generated PI controller        */
      u = ControllerModel_Y.u;
      if (u < 0.0) u = 0.0;                    /* PWM duty saturates at 0..3.3 V */
      if (u > VOLT_REF) u = VOLT_REF;
    } else if (strcmp(scenario, "chirp") == 0) {
      u = chirp(t);
    } else {
      u = 1.0;                                 /* unit step at t = 0             */
    }

    PlantModel_U.u = (real_T)u;
    PlantModel_step();
    double y = PlantModel_Y.y;
    double y_rk4 = 25.0 * PlantModel_X.TransferFcn_CSTATE[1];
    y_prev = y;

    if (closed) fprintf(f, "%.6f,%.9g,%.9g,%.9g,%.9g,%.9g\n", t, r, e, u, y, y_rk4);
    else        fprintf(f, "%.6f,%.9g,%.9g,%.9g\n", t, u, y, y_rk4);
  }

  fclose(f);
  printf("[harness] %-10s %d steps @ %.0f ms (ode4, real_T=%zu bytes) -> %s\n",
         scenario, n, TS * 1000.0, sizeof(real_T), argv[2]);
  return 0;
}
