/* Host SIL harness: runs the Embedded Coder generated reference, controller
 * and plant exactly as the two STM32 timer callbacks chain them, but
 * lock-stepped at the model sample time on the host. Writes CSV to stdout. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "PlantModel.h"
#include "ControllerModel.h"
#include "ReferenceInputModel.h"

#define SAMPLE_TIME 0.05

static double plant_state_output(void)
{
  return 25.0 * PlantModel_X.TransferFcn_CSTATE[1];
}

static void open_loop(int steps, double u_step)
{
  int k;
  PlantModel_initialize();
  printf("k,t,u,y,y_state\n");
  for (k = 0; k < steps; k++) {
    printf("%d,%.4f,%.9g,%.9g,%.9g\n", k, k * SAMPLE_TIME, u_step,
           (double)PlantModel_Y.y, plant_state_output());
    PlantModel_U.u = (real_T)u_step;
    PlantModel_step();
  }
}

static void closed_loop(int steps)
{
  int k;
  float y = 0.f, r, e, u;
  ReferenceInputModel_initialize();
  ControllerModel_initialize();
  PlantModel_initialize();
  printf("k,t,r,y,e,u,y_state\n");
  for (k = 0; k < steps; k++) {
    /* controller/Core/Src/main.c HAL_TIM_PeriodElapsedCallback */
    ReferenceInputModel_step();
    r = ReferenceInputModel_Y.r;
    e = r - y;
    ControllerModel_U.e = e;
    ControllerModel_step();
    u = ControllerModel_Y.u;
    printf("%d,%.4f,%.9g,%.9g,%.9g,%.9g,%.9g\n", k, k * SAMPLE_TIME, r, y, e,
           u, plant_state_output());

    /* plant_simulator/Core/Src/main.c HAL_TIM_PeriodElapsedCallback */
    PlantModel_U.u = u;
    PlantModel_step();
    y = PlantModel_Y.y;
  }
}

int main(int argc, char **argv)
{
  int steps = argc > 2 ? atoi(argv[2]) : 320;
  if (argc > 1 && strcmp(argv[1], "open") == 0) {
    open_loop(steps, argc > 3 ? atof(argv[3]) : 1.0);
  } else if (argc > 1 && strcmp(argv[1], "closed") == 0) {
    closed_loop(steps);
  } else {
    fprintf(stderr, "usage: %s open|closed [steps] [u_step]\n", argv[0]);
    return 2;
  }
  return 0;
}
