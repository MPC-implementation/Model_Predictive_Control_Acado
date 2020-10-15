/*
 *  ACADO MPC code generator for kinematic bicycle model
 * http://acado.sourceforge.net/doc/html/d4/d26/example_013.html
 * http://acado.sourceforge.net/doc/html/db/daf/cgt_getting_started.html
 */

#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
{
   
        // INTRODUCE THE VARIABLES (acadoVariables.x):
        // -------------------------
        DifferentialState x; 
        DifferentialState y;
        DifferentialState v;	
        DifferentialState phi;

	OnlineData coeff0, coeff1, coeff2, coeff3;
        
        Control a;
        Control delta;        
        //Control deltarate;        
                
        double L = 2.67;	  // vehicle wheel base
        double dt = 0.1; // sampling time for discrete-time system

        // DEFINE A DIFFERENTIAL EQUATION:
        // -------------------------------
        DifferentialEquation f;
        // model equations
        f << dot(x) == v*cos(phi);
        f << dot(y) == v*sin(phi);
        f << dot(v) == a;
        f << dot(phi) == v*tan(delta)/L;  
        //f << dot(delta) == deltarate;       
  
        //  
        // Weighting matrices and reference functions (acadoVariables.y)
        //
	auto y_dev = coeff0 + coeff1 * x + coeff2 * x * x + coeff3 * x * x * x;
	auto angle = atan(coeff1 + 2*coeff2*x + 3*coeff3*x*x);

        Function rf, rfN;

	rf << y_dev - y;
	rf << angle - phi;
	rf << v - 60.0; 
	rfN << y_dev - y;
	rfN << angle - phi;
	rfN << v - 60.0;

        const int N  = 20;
        const double Ts = 0.1;

        // Provide defined weighting matrices:
        BMatrix W = eye<bool>(rf.getDim());
        BMatrix WN = eye<bool>(rfN.getDim());

        OCP ocp(0, N * Ts, N);

        ocp.subjectTo( f );
        // control constraints
        ocp.subjectTo( -1.0 <= a <= 1.0 );
        //ocp.subjectTo( 0.05*0.05 <= v*v <= 1.0*1.0 );
        ocp.subjectTo( -0.4363 <= delta <= 0.4363 );        
        //ocp.subjectTo( -M_PI/4 <= deltarate <= M_PI/4 );
	ocp.setNOD(4);
 
        ocp.minimizeLSQ(W, rf);
        ocp.minimizeLSQEndTerm(WN, rfN);

        //
        // Export the code:
        //
        OCPexport mpc( ocp );
        
        mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
        mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
        mpc.set(INTEGRATOR_TYPE, INT_RK4);
        mpc.set(NUM_INTEGRATOR_STEPS, 1 * N);
        mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
        mpc.set(QP_SOLVER, QP_QPOASES);
        mpc.set(MAX_NUM_QP_ITERATIONS, 500);
        mpc.set(HOTSTART_QP, YES);        
        mpc.set(SPARSE_QP_SOLUTION, CONDENSING);        
        //	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);
        mpc.set(GENERATE_TEST_FILE, YES);
        mpc.set(GENERATE_MAKE_FILE, YES);
        mpc.set(GENERATE_MATLAB_INTERFACE, YES);
        mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);

        if (mpc.exportCode( "simple_mpc_export" ) != SUCCESSFUL_RETURN)
                exit( EXIT_FAILURE );

        mpc.printDimensionsQP( );


        return EXIT_SUCCESS;
}
