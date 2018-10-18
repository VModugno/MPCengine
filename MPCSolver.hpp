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
		void logToFile();

		// GET function
		int getStateDim(){return this->n;};
		int getControlDim(){return this->m;};
		int getOutputDim(){return this->p;};



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
		qpOASES::QProblem qp;

};



#endif
