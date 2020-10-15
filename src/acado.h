#ifndef ACADO_H
#define ACADO_H

#include <stdio.h>
#include <vector>
#include "../acado_mpc_export/acado_common.h"
#include "../acado_mpc_export/acado_auxiliary_functions.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
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

typedef struct{
double a[N+1];
double delta[N+1];
} ctrl;

void init_weight();
void run_mpc_acado(const std::vector<double>& states, 
		const Eigen::Matrix<double, -1, 1>& coeff,
		ctrl& c);
void init_acado();
void create_reference(
		const double& velocity,
		const Eigen::Matrix<double, -1, 1>& coeff,
		std::vector<std::vector<double>>& ref);
#endif
