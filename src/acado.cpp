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
vector<vector<double>> init_acado()
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
	acado_preparationStep();
	vector<double> control_output_acceleration;
	vector<double> control_output_steering;
	for (int i = 0; i < ACADO_N; ++i)
	{
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				control_output_acceleration.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
			else // only two output. therefore i == 1 means steering command output
			{
				control_output_steering.push_back(acadoVariables.u[i * ACADO_NU + j]);
			}
		}
	}
	return {control_output_acceleration, control_output_steering};
}
vector<vector<double>> run_mpc_acado(vector<double> states, vector<double> ref_states, vector<vector<double>> previous_u)
{
	/* Some temporary variables. */
	int i, iter;
	acado_timer t;

	//previou
	// acado_shiftStates(2, 0, 0);
	// acado_shiftControls(0);

	for (i = 0; i < NX * (N + 1); ++i)  
	{
		acadoVariables.x[ i ] = (real_t) states[i];
	}
	
	int u_cnt = 0;
	for (int i = 0; i < ACADO_N; ++i)
	{
		for (int j = 0; j < ACADO_NU; ++j)
		{
			acadoVariables.u[u_cnt] = (real_t) previous_u[j][i];
			u_cnt++;
		}
		printf("\n");
	}
	// for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;
	// acado_printControlVariables();

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  
	{
		acadoVariables.y[ i ] = (real_t) ref_states[i];
	}
	for (i = 0; i < NYN; ++i) 	
	{
		acadoVariables.yN[i] = (real_t) ref_states[NY * (N-1) + i];
	}


	// acado_printDifferentialVariables();
	// acado_printControlVariables();
	/* MPC: initialize the current state feedback. */
	// #if ACADO_INITIAL_STATE_FIXED
	// 	for (i = 0; i < NX; ++i)
	// 		acadoVariables.x0[i] = 0.1;
	// #endif

	if (VERBOSE)
		acado_printHeader();

	// /* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic(&t);

	/* The "real-time iterations" loop. */

	/* Perform the feedback step. */
	acado_feedbackStep();
	// acado_preparationStep();

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

	acado_printDifferentialVariables();
	acado_printControlVariables();

	vector<double> control_output_acceleration;
	vector<double> control_output_steering;
	real_t *u = acado_getVariablesU();
	printf("u: %lf \n", u[0]);

	for (int i = 0; i < ACADO_N; ++i)
	{
		for (int j = 0; j < ACADO_NU; ++j)
		{
			if (j == 0)
			{
				control_output_acceleration.push_back((double) u[i * ACADO_NU + j]);
			}
			else // only two output. therefore i == 1 means steering command output
			{
				control_output_steering.push_back((double)u[i * ACADO_NU + j]);
			}
		}
	}
	return {control_output_acceleration, control_output_steering};
}

vector<double> calculate_ref_states(const Eigen::VectorXd &coeff, const double &reference_v)
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
	for (int i = 0; i < N - 1; i++)
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
	vector<vector<double>> ref_states = {ref_x, ref_y, ref_v, ref_yaw};
	vector<double> result;
	for (int i = 0; i < ACADO_N; ++i)
	{
		for (int j = 0; j < NY; ++j)
		{
			result.push_back(ref_states[j][i]);
		}
	}
	return result;
}
vector<double> motion_prediction(const vector<double> &cur_states, const vector<vector<double>> &prev_u)
{
	vector<double> old_acceleration_cmd = prev_u[0];
	vector<double> old_steering_cmd = prev_u[1];
	vector<vector<double>> predicted_states;
	predicted_states.push_back(cur_states);
	// for (int i = 0; i < N ; i++)
	// {
	// 	printf("i: %d, old_acceleration_cmd: %lf \n", i, old_acceleration_cmd[i]);
	// 	printf("i: %d, old_steering_cmd: %lf \n", i, old_steering_cmd[i]);
	// }
	for (int i = 0; i < N; i++)
	{
		vector<double> cur_state = predicted_states[i];
		// yaw angle compensation of overflow
		if (cur_state[3] > M_PI)
		{
			cur_state[3] -= 2 * M_PI;
		}
		if (cur_state[3] < -M_PI)
		{
			cur_state[3] += 2. * M_PI;
		}
		vector<double> next_state = update_states(cur_state, old_acceleration_cmd[i], old_steering_cmd[i]);
		predicted_states.push_back(next_state);
	}
	// printf("-------- motion prediction -------- \n");
	// for (int i = 0; i < N + 1; i++)
	// {
	// 	printf("i: %d, x: %lf, y: %lf, v: %lf, yaw: %lf\n", i, predicted_states[i][0], predicted_states[i][1], predicted_states[i][2], predicted_states[i][3]);
	// }
	vector<double> result;
	for (int i = 0; i < (ACADO_N + 1); ++i)
	{
		for (int j = 0; j < NY; ++j)
		{
			result.push_back(predicted_states[i][j]);
		}
	}
	return result;
}

vector<double> update_states(vector<double> state, double acceleration_cmd, double steering_cmd)
{
	// based on kinematic model
	double x0 = state[0];
	double y0 = state[1];
	double v0 = state[2];
	double yaw0 = state[3];

	double x1 = x0 + v0 * cos(yaw0) * Ts;
	double y1 = y0 + v0 * sin(yaw0) * Ts;
	double v1 = v0 + acceleration_cmd * Ts;
	double yaw1 = yaw0 + v0 / Lf * steering_cmd * Ts;
	return {x1, y1, v1, yaw1};
}