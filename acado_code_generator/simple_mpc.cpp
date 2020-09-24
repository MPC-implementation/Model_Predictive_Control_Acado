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
        //DifferentialState delta;        
        
   
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
  
  /*
        // DEFINE A DISCRETE-TIME SYTSEM:
        // -------------------------------
        DiscretizedDifferentialEquation  f(dt);       
        f << next(x) == v*cos(phi)*dt;
        f << next(y) == v*sin(phi)*dt;
        f << next(v) == a*dt;
        f << next(phi) == v*tan(delta)/L*dt;
    */ 
        //  
        // Weighting matrices and reference functions (acadoVariables.y)
        //

        Function rf;
        Function rfN;

        rf  << x << y << v << phi; // << a << deltarate;
        rfN << x << y << v << phi;

        const int N  = 7;
        const int Ni = 6;
        const double Ts = 0.1;

        // Provide defined weighting matrices:
        BMatrix W = eye<bool>(rf.getDim());
        BMatrix WN = eye<bool>(rfN.getDim());
        
        //DMatrix W = eye<double>( rf.getDim() );
        //DMatrix WN = eye<double>( rfN.getDim() );
        /*W(0,0)=0.1;
        WN(0,0)=1.0;
        
        W(1,1)=0.1;
        WN(1,1)=1.0;  
        
        W(2,2)=0.05;
        WN(2,2)=0.05;*/       

        //DMatrix WN = eye<double>( rfN.getDim() ) * N;
                
        //printf("WN %dx%d - dim:%d\n", WN.getNumRows(), WN.getNumCols(), rf.getDim());
        //for (int i=0; i < rf.getDim(); i++){
        //  WN(i,i) = W(i,i);
        //}

        OCP ocp(0, N * Ts, N);

        ocp.subjectTo( f );
        // control constraints
        ocp.subjectTo( -1.0 <= a <= 1.0 );
        //ocp.subjectTo( 0.05*0.05 <= v*v <= 1.0*1.0 );
        ocp.subjectTo( -0.4363 <= delta <= 0.4363 );        
        //ocp.subjectTo( -M_PI/4 <= deltarate <= M_PI/4 );
 
        // obstacle contraints
        // for(int i = 0; i < NUM_OBST; i++){
        //   ocp.subjectTo(sqrt((x[0]-obst[i][0])*(x[0]-obst[i][0]) + (x[2]-obst[i][1])*(x[2]-obst[i][1])) >= OBST_THRS);
        // }

        ocp.minimizeLSQ(W, rf);
        ocp.minimizeLSQEndTerm(WN, rfN);

        //
        // Export the code:
        //
        OCPexport mpc( ocp );
        
        mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
        //mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);        
        mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
        //mpc.set(INTEGRATOR_TYPE, INT_RK4);
        mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
        mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
        mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
        //	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
        mpc.set(QP_SOLVER, QP_QPOASES);
        //	mpc.set(QP_SOLVER, QP_FORCES);
        //	mpc.set(MAX_NUM_QP_ITERATIONS, 20);
        mpc.set(HOTSTART_QP, YES);        
        //	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);        
        //	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);
        mpc.set(GENERATE_TEST_FILE, YES);
        mpc.set(GENERATE_MAKE_FILE, YES);
        mpc.set(GENERATE_MATLAB_INTERFACE, YES);
        //	mpc.set(USE_SINGLE_PRECISION, YES);
        mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
        //mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
        //	mpc.set(CG_USE_OPENMP, YES);
        // NOTE: This is crucial for export of MHE!
	      //mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
	      mpc.set(FIX_INITIAL_STATE, YES);

        if (mpc.exportCode( "simple_mpc_export" ) != SUCCESSFUL_RETURN)
                exit( EXIT_FAILURE );

        mpc.printDimensionsQP( );


        /*
        // DEFINE LEAST SQUARE FUNCTION:
        // -----------------------------
        // path constraints
        Function h;

        h << x;
        h << y;
        h << v;
        h << phi;
        h << delta;
        h << a;
        h << dotdelta;

        DMatrix Q(7,7); // LSQ coefficient matrix
        Q.setIdentity();
        Q(0,0) = 1.0;
        Q(1,1) = 1.0;
        Q(2,2) = 1.0;
        Q(3,3) = 1.0;
        Q(3,3) = 1.0;
        Q(4,4) = 1.0;
        Q(5,5) = 1.0;
        Q(6,6) = 1.0;    

        // terminal constraints
        DVector r(5);
        r.setAll( 0.0 );

        // DEFINE AN OPTIMAL CONTROL PROBLEM:
        // ----------------------------------
        const double t_start = 0.0;
        const double t_end   = 10.0;
        const int N = 10;

        OCP ocp( t_start, t_end, N );

        ocp.subjectTo( f );

        // control constraints
        ocp.subjectTo( -5 <= a <= 5 );
        ocp.subjectTo( -3.14 <= dotdelta <= 3.14 ); 

        ocp.minimizeLSQ( Q, h, r ); 

        // SETTING UP THE (SIMULATED) PROCESS:
        // -----------------------------------
        OutputFcn identity;
        DynamicSystem dynamicSystem( f,identity );

        Process process( dynamicSystem,INT_RK45 );

        // SETTING UP THE MPC CONTROLLER:
        // ------------------------------
        RealTimeAlgorithm alg( ocp,0.1 ); // sampling time
        alg.set( MAX_NUM_ITERATIONS, 2 );

        StaticReferenceTrajectory zeroReference("ref.txt");

        Controller controller( alg,zeroReference );


        // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
        // ----------------------------------------------------------
        SimulationEnvironment sim( 0.0,10.0,process,controller );

        DVector x0(5);
        x0(0) = -2.0;
        x0(1) = 0.0;
        x0(2) = 0.0;
        x0(3) = 0.0;
        x0(4) = 0.0;

        if (sim.init( x0 ) != SUCCESSFUL_RETURN)
                exit( EXIT_FAILURE );
        if (sim.run( ) != SUCCESSFUL_RETURN)
                exit( EXIT_FAILURE );

        // ...AND PLOT THE RESULTS
        // ----------------------------------------------------------
        VariablesGrid sampledProcessOutput;
        sim.getSampledProcessOutput( sampledProcessOutput );

        VariablesGrid feedbackControl;
        sim.getFeedbackControl( feedbackControl );

        GnuplotWindow window;
        window.addSubplot( sampledProcessOutput(0), "x Position [m]" );
        window.addSubplot( sampledProcessOutput(1), "y Position [m]" );
        window.addSubplot( sampledProcessOutput(2), "Velocity [m/s]" );
        window.addSubplot( sampledProcessOutput(3), "Phi [rad]" );
        window.addSubplot( sampledProcessOutput(4), "Delta [rad]" );
        window.addSubplot( feedbackControl(0),      "Acceleration [m/ss]" );
        window.addSubplot( feedbackControl(1),      "Steering [rad/ss]" );
        window.plot( );
        
 */

        return EXIT_SUCCESS;
}



