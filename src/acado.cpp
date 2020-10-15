#include "acado.h"
#include <iostream>
#include <limits>
#include <unsupported/Eigen/Polynomials>
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
void init_weight() {
double w_cte = 0.001;
double w_angle = 0.0;
double w_velo = 0.2;
for (int i = 0; i < N; i++) {
    // Setup diagonal entries
    acadoVariables.W[NY*NY*i + (NY+1)*0] = w_cte;
    acadoVariables.W[NY*NY*i + (NY+1)*1] = w_angle;
    acadoVariables.W[NY*NY*i + (NY+1)*2] = w_velo;
}
  acadoVariables.WN[(NYN+1)*0] = w_cte;
  acadoVariables.WN[(NYN+1)*1] = w_angle;
  acadoVariables.WN[(NYN+1)*2] = w_velo;

}
/* A template for testing of the solver. */
void init_acado()
{
	/* Initialize the solver. */
	acado_initializeSolver();
	for (int i = 0; i < NX * (N + 1); ++i)
		acadoVariables.x[i] = 0.0;
	for (int i = 0; i < NU * N; ++i)
		acadoVariables.u[i] = 0.0;

	// /* Initialize the measurements/reference. */
	// for (i = 0; i < NY * N; ++i)
	// 	acadoVariables.y[i] = 0.0;
	// for (i = 0; i < NYN; ++i)
	// 	acadoVariables.yN[i] = 0.0;
	init_weight();
}
void run_mpc_acado(const std::vector<double>& states, 
		const Eigen::Matrix<double, -1, 1>& coeff,
		ctrl& c)
{
	/* Some temporary variables. */
	int i, iter;
	acado_timer t;

	acado_shiftStates(1, 0, 0);
	acado_shiftControls(0);

	for (i = 0; i < NX; ++i)
	{
		acadoVariables.x[i] = states[i];
		acadoVariables.x0[i] = states[i];
	}

	// Feed coeff
	for (int i=0; i<=NOD * N; i += NOD){
		acadoVariables.od[i] = coeff[0];
		acadoVariables.od[i+1] = coeff[1];
		acadoVariables.od[i+2] = coeff[2];
		acadoVariables.od[i+3] = coeff[3];
	}

	/* Get the time before start of the loop. */
	acado_tic(&t);

	
	/* Prepare first step */
	acado_preparationStep();

	/* Perform the feedback step. */
	acado_feedbackStep();

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
	std::cout<<"cost: "<<acado_getObjective()<<std::endl;

	acado_printDifferentialVariables();
	acado_printControlVariables();

	for (i=0; i <=N; i++) {
		c.a[i] = acadoVariables.u[2*i];
		c.delta[i] = acadoVariables.u[2*i+1];
	}
}

void create_reference(
		const double& velocity,
		const Eigen::Matrix<double, -1, 1>& coeff,
		std::vector<std::vector<double>>& ref) {
// state: x, y, v, psi

    static const double dt = 0.1;
    double velo = std::max(velocity, 1.0);

    double prev_d = 0.0;
    for(int i=1; i<N; ++i) {
	double d = i * dt * velo;

	// https://eigen.tuxfamily.org/dox/unsupported/classEigen_1_1PolynomialSolver.html#details
	Eigen::Matrix<double, 5, 1> poly;
	poly << -d, coeff[0], coeff[1]/2.0, coeff[2]/3.0, coeff[3]/4.0;
	Eigen::PolynomialSolver<double, 4> solver(poly);
	std::vector<double> roots;
	solver.realRoots(roots);
	// find positive min
	double ans_x = std::numeric_limits<double>::max();
	for(auto &i : roots) if(i>=0.01) ans_x = std::min(ans_x, i);

	double x = ans_x;
	double y = coeff[0] + coeff[1] * x + coeff[2] * pow(x,2) + coeff[3] * pow(x,3);
	double psi = atan(coeff[1] + 2.0 * coeff[2] * x + 3.0 * coeff[3] * pow(x, 2));

	ref[i] = std::vector<double>{x, y, velo, psi};
	std::cout<<"? "<<d<<" "<<x<<" "<<y<<" "<<velo<<" "<<psi<<std::endl;
    }
}

void get_control(std::vector<std::vector<double>>& ctrl) {
	for(int i=0; i<N; i++) {
		for (int j=0; j<NU; j++){
			ctrl[i][j] = acadoVariables.u[i*NU+j];
		}
	}
}

