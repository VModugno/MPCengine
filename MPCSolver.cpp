#include "MPCSolver.hpp"
#include <stdlib.h>
#include <chrono>

using namespace mpcSolver;
using namespace std::chrono;


MPCSolver::MPCSolver(int n,int m,int p,int N){
	// Set up parameters
	this->n = n;
	this->m = m;
	this->p = p;
    this->N = N;
}


Eigen::VectorXd MPCSolver::initSolver(double * x0)
{
	int nVariables   = this->N * this->m;
	int nConstraints = this->N * this->N_constr;
    // solver constructor
	this->qp=qpOASES::QProblem(nVariables,nConstraints);
	// qpoases option
	qpOASES::Options options;
    options.setToMPC();
    options.printLevel=qpOASES::PL_NONE;
    qpOASES::int_t nWSR = 300;
    // fitness matrices
	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];
	// constraints matrices
	qpOASES::real_t A[nConstraints*nVariables];
	qpOASES::real_t lbA[nConstraints];
	qpOASES::real_t ubA[nConstraints];
	// solution
	qpOASES::real_t xOpt[nVariables];
	Eigen::VectorXd decisionVariables(this->m);

    // compute components
	compute_H(H);
	compute_g(g,x0);
	compute_A(A);
	compute_ub(ubA,x0);

	// solve optimization problem
	returnValue ret = qp.init(H,g,A,NUL,NULL,NULL,ubA,this->nWSR,NULL);
    // get results
	qp.getPrimalSolution(xOpt)

	for(int i=0;i<this->m;++i){
		decisionVariables(i) = xOpt[i];
	}

    return decisionVariables;
}



Eigen::VectorXd MPCSolver::solveQP(double *Xi) {

	int nVariables   = this->N*this->m;
	int nConstraints = this->N*this->N_constr;

	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];

	qpOASES::real_t A[nConstraints*nVariables];
	qpOASES::real_t lb[nConstraints];
	qpOASES::real_t ub[nConstraints];

	qpOASES::real_t xOpt[nVariables];
	Eigen::VectorXd decisionVariables(this->m);

	// compute components
	compute_H(H);
	compute_g(g,xi);
	compute_A(A);
	compute_ub(ubA,xi);
	// compute solutions
	qp.hotstart(g,NULL,NULL,NULL,ubA,this->nWSR,NULL);
    // compute
	qp.getPrimalSolution(xOpt);
	for(int i=0;i<m;++i){
		decisionVariables(i) = xOpt[i];
	}

	return decisionVariables;

}



void MPCSolver::logToFile() {
}




