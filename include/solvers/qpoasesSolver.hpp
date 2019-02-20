#ifndef QPOASESSOLVER_HPP
#define QPOASESSOLVER_HPP


#include "qpOASES.hpp"
#include "AbsSolver.hpp"
#include "qpOASESFunction.h"


class qpoasesSolver  : public AbsSolver {
	public:
		//qpoasesSolver(int n,int m,int p,int N, int N_constr,std::string type,std::string solver,bool direct_solution = false);
		qpoasesSolver(const std::string filename,bool direct_solution = false);
		// initialization
		// i have introduced initDim in order to manage the case with state_machine mpc that provides
		// dimensions that are arrays and not
	    void            initDim(pt::ptree & tree);
		Eigen::VectorXd initSolver(Eigen::VectorXd x0_in,Eigen::VectorXd  x0_ext,ProblemDetails & pd);
		Eigen::VectorXd initSolver(double * x0_in,double * x0_ext,ProblemDetails & pd);
		// Solve
		Eigen::VectorXd solveQP(Eigen::VectorXd xi_in,Eigen::VectorXd  xi_ext,ProblemDetails & pd);
		Eigen::VectorXd solveQP(double * xi_in,double * xi_ext,ProblemDetails & pd);
		// Log
		void plotInfoQP();
		void logToFile(std::string data2log);
	    // Analysis of the QPoases result
		bool GetVerySimpleStatus(qpOASES::returnValue ret,bool debug);
		// destructor
		~qpoasesSolver(){};

	private:
		// Compute MAtrix for the MPC QP batch problem
	    void computeMatrix(qpOASES::real_t  H[],qpOASES::real_t g[],qpOASES::real_t A[],qpOASES::real_t ubA[],double * xi_in,double * xi_ext,ProblemDetails & pd);
	    // solver parameters
	    int nVariables_batch;                // total number of variables over the prediction windows
	    int nConstraints_batch;              // total number of constraints over the prediction windows
	    std::string type;
	    qpOASES::int_t nWSR;
		qpOASES::int_t original_nWSR;   // i need this variables in order to restore the value of nwrs (that is overwritten by the method)
		//double mpcTimeStep          = 100;
		//double controlTimeStep      = 100;

		// Parameters for the current iteration
		//double simulationTime       = 100;
		//int mpcIter=100,controlIter = 100;
		bool direct_solution;        //= true;   // with this flag i activate the solution recalculation each time and I use qp instead of sqp

		// Quadratic problem
		qpOASES::SQProblem sqp;
		qpOASES::QProblem qp;

		// options
		qpOASES::Options options;

};



#endif
