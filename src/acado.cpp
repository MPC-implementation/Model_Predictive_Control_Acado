#include "acado.h"

/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */

/*

IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/
/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
using namespace std;
/* A template for testing of the solver. */
void init_acado()
{
	/* Initialize the solver. */
	acado_initializeSolver();
	for (int i = 0; i < NX * (N + 1); ++i)
		acadoVariables.x[i] = 0.0;
	for (int i = 0; i < NU * N; ++i)
		acadoVariables.u[i] = 0.0;
	for (int i = 0; i < NY * N; ++i)
		acadoVariables.y[i] = 0.0;
	for (int i = 0; i < NYN; ++i)
		acadoVariables.yN[i] = 0.0;
	// /* Initialize the measurements/reference. */
	// for (i = 0; i < NY * N; ++i)
	// 	acadoVariables.y[i] = 0.0;
	// for (i = 0; i < NYN; ++i)
	// 	acadoVariables.yN[i] = 0.0;
}
void run_mpc_acado(std::vector<double> states, std::vector<vector<double>> ref_states)
{
	/* Some temporary variables. */
	int i, iter;
	acado_timer t;

	//previou
	acado_shiftStates(2, 0, 0);
	acado_shiftControls(0);
	for (i = 0; i < NX * (N + 1); ++i)
	{
		acadoVariables.x[i] = 0.0;
	}
	for (i = 0; i < NX; ++i)
	{
		acadoVariables.x0[i] = states[i];
	}
	for (i = 0; i < NU; ++i)
		acadoVariables.u[i] = 0; // previous u

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i) // N = Horizon, NY = Reference states
		acadoVariables.y[i] = 0.0;
	for (i = 0; i < NYN; ++i)
		acadoVariables.yN[i] = 0.0;

	/* MPC: initialize the current state feedback. */
	// #if ACADO_INITIAL_STATE_FIXED
	// 	for (i = 0; i < NX; ++i)
	// 		acadoVariables.x0[i] = 0.1;
	// #endif

	if (VERBOSE)
		acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic(&t);

	/* The "real-time iterations" loop. */

	/* Perform the feedback step. */
	acado_feedbackStep();

	/* Apply the new control immediately to the process, first NU components. */

	if (VERBOSE)
		printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT());

	/* Optional: shift the initialization (look at acado_common.h). */
	/* acado_shiftStates(2, 0, 0); */
	/* acado_shiftControls( 0 ); */

	/* Read the elapsed time. */
	real_t te = acado_toc(&t);

	if (VERBOSE)
		printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if (!VERBOSE)
		printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	// acado_printDifferentialVariables();
	// acado_printControlVariables();
}

vector<vector<double>> calculate_ref_states(const Eigen::VectorXd &coeff, const double &reference_v)
{
	vector<double> ref_x;
	vector<double> ref_y;
	vector<double> ref_yaw;
	vector<double> ref_v;
	double x0 = 0;
	double y0 = coeff[0] + coeff[1] * x0 + coeff[2] * pow(x0, 2) + coeff[3] * pow(x0, 3);
	double yaw0 = atan(coeff[1] + 2 * coeff[2] * x0 + 3 * coeff[3] * pow(x0, 2));
	ref_x.push_back(x0);
	ref_y.push_back(y0);
	ref_yaw.push_back(yaw0);
	ref_v.push_back(reference_v);
	double d = reference_v * Ts;
	for (int i = 0; i < N-1; i++)
	{
		// current states
		double cur_x = ref_x[i];
		double cur_y = ref_y[i];
		double cur_yaw = ref_yaw[i];
		double dx = d * cos(ref_yaw[i]);
		double dy = d * sin(ref_yaw[i]);
		ref_x.push_back(ref_x[i] + dx);
		ref_y.push_back(ref_y[i] + dy);
		double next_yaw = atan(coeff[1] + 2 * coeff[2] * ref_x[i + 1] + 3 * coeff[3] * pow(ref_x[i + 1], 2));
		ref_yaw.push_back(next_yaw);
		ref_v.push_back(reference_v);
	}
	for (int i = 0; i < N; i++)
	{
		printf("i: %d, x: %.2f, y: %.2f, v: %.2f, yaw: %.2f\n", i, ref_x[i], ref_y[i], ref_v[i], ref_yaw[i]);
	}
	return {ref_x, ref_y, ref_v, ref_yaw};
}
