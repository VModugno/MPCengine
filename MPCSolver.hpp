#ifndef MPCSOLVER_HPP
#define MPCSOLVER_HPP
#include <vector>
#include <Eigen/Core>
#include "qpOASES/qpOASES.hpp"
#include "current_function/QPoasesFunction.h"

namespace mpcSolver{

    class MPCSolver{
	public:
        MPCSolver(int n,int m,int p,int N, Int N_constr,H_dim,g_dim,A_dim, ub_dim);
        //
        void initSolver();
        // Solve
        Eigen::VectorXd solveQP();
        // Log
        void logToFile();

	public:

        // to remove (because Im gonna compute them with the more general problem feature (solver independent))
        int H_dim;
		int g_dim;
		int A_dim;
		int ub_dim;

	private:

        // Constant parameters
        int n;        // state   space dim
		int m;        // control space dim
		int p;        // output  space dim
		int N;        // prediction window
		Int N_constr; // number of constraints
        
        double mpcTimeStep;
        double controlTimeStep;

        // Parameters for the current iteration
        double simulationTime;
        int mpcIter,controlIter;


	    // Quadratic problem
	    qpOASES::QProblem qp;

   };

}

#endif
