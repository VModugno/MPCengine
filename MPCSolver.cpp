#include "MPCSolver.hpp"
#include <stdlib.h>
#include <chrono>

//using namespace mpcSolver;
//using namespace std::chrono;


MPCSolver::MPCSolver(int n,int m,int p,int N, int N_constr){ //int H_dim,int g_dim,int A_dim, int ub_dim){
	// Set up parameters
	this->n           = n;
	this->m           = m;
	this->p           = p;
    this->N           = N;
    this->N_constr    = N_constr;
    int nVariables    = this->N * this->m;
    int nConstraints  = this->N * this->N_constr;
    this->qp          = qpOASES::QProblem(nVariables,nConstraints);
}


Eigen::VectorXd MPCSolver::initSolver(double * x0)
{
	int nVariables   = this->N * this->m;
	int nConstraints = this->N * this->N_constr;
	// qpoases option
	qpOASES::Options options;
    options.setToMPC();
    options.printLevel=qpOASES::PL_NONE;
    // fitness matrices
	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];
	// constraints matrices
	qpOASES::real_t A[nConstraints*nVariables];
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
	qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
    // get results
	qp.getPrimalSolution(xOpt);

	for(int i=0;i<this->m;++i){
		decisionVariables(i) = xOpt[i];
	}

    return decisionVariables;
}



Eigen::VectorXd MPCSolver::solveQP(double *xi) {

	int nVariables   = this->N*this->m;
	int nConstraints = this->N*this->N_constr;

	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];

	qpOASES::real_t A[nConstraints*nVariables];
	qpOASES::real_t lb[nConstraints];
	qpOASES::real_t ubA[nConstraints];

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




