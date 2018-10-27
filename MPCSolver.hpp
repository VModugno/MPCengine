#ifndef MPCSOLVER_HPP
#define MPCSOLVER_HPP
#include <vector>
#include <Eigen/Core>
#include <iostream>
#include "qpOASES.hpp"
#include "qpOASESFunction.h"





class MPCSolver{
	public:
		MPCSolver(int n,int m,int p,int N, int N_constr,std::string type,std::string solver);
		MPCSolver(const std::string filename);
		//
		Eigen::VectorXd initSolver(Eigen::VectorXd x0_in);
		Eigen::VectorXd initSolver(double * x0);
		// Solve
		Eigen::VectorXd solveQP(Eigen::VectorXd xi_in);
		Eigen::VectorXd solveQP(double * xi);
		// Log
		void plotInfoQP();

		void logToFile(std::string data2log);

		// GET function
		int getStateDim(){return this->n;};
		int getControlDim(){return this->m;};
		int getOutputDim(){return this->p;};

	    // Analysis of the QPoases result
		bool GetVerySimpleStatus(qpOASES::returnValue ret,bool debug = false){

			bool result;
			int r = qpOASES::getSimpleStatus(ret);


			if(r == 0){
				if(debug)
					std::cout << "QP solved" << std::endl;
				result = true;
			}
			else if(r == 1){
				if(debug)
					std::cout << "QP could not be solved within the given number of iterations" << std::endl;
				result =false;
			}
			else if(r == -1){
				if(debug)
					std::cout << "QP could not be solved due to an internal error" << std::endl;
				result =false;
			}
			else if(r == -2){
				if(debug)
					std::cout << "QP is infeasible and thus could not be solved" << std::endl;
				result =false;
			}
			else if(r == -3){
				if(debug)
					std::cout << "QP is unbounded and thus could not be solved" << std::endl;
				result =false;
			}

			return result;
		}



	public:

		// to remove (because Im gonna compute them with the more general problem feature (solver independent))
		//int H_dim;
		//int g_dim;
		//int A_dim;
		//int ub_dim;

	private:

		// problem parameters
		int n;        // state   space dim
		int m;        // control space dim
		int p;        // output  space dim
		int N;        // prediction window
		int N_constr; // number of constraints
		std::string  type;
		std::string  solver;

		// solver parameters
		qpOASES::int_t nWSR         = 3000;
		double mpcTimeStep          = 100;
		double controlTimeStep      = 100;

		// Parameters for the current iteration
		double simulationTime       = 100;
		int mpcIter=100,controlIter = 100;


		// Quadratic problem
		qpOASES::SQProblem qp;

};



#endif
