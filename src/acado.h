#ifndef ACADO_H
#define ACADO_H

#include <stdio.h>
#include <vector>
#include "../acado_mpc_export/acado_common.h"
#include "../acado_mpc_export/acado_auxiliary_functions.h"
/* Some convenient definitions. */
#define NX ACADO_NX		/* Number of differential state variables.  */
#define NXA ACADO_NXA /* Number of algebraic variables. */
#define NU ACADO_NU		/* Number of control inputs. */
#define NOD ACADO_NOD /* Number of online data values. */

#define NY ACADO_NY		/* Number of measurements/references on nodes 0..N - 1. */
#define NYN ACADO_NYN /* Number of measurements/references on node N. */

#define N ACADO_N /* Number of intervals in the horizon. */

#define NUM_STEPS 10 /* Number of real-time iterations. */
#define VERBOSE 1		 /* Show iterations: 1, silent: 0.  */

void run_mpc_acado(std::vector<double> states, std::vector<double> path);
void init_acado();
#endif