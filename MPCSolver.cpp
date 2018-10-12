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


void MPCSolver::initSolver()
{




}



Eigen::VectorXd MPCSolver::solveQP() {

	int nVariables   = this->N*this->m;
	int nConstraints = this->N*this->N_constr;

	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];

	qpOASES::real_t A[nConstraints*nVariables];
	qpOASES::real_t lb[nConstraints];
	qpOASES::real_t ub[nConstraints];


	qpOASES::real_t xOpt[nVariables];

	qpOASES::Options options;
	options.setToMPC();
	options.printLevel=qpOASES::PL_NONE;
	qpOASES::int_t nWSR = 300;

	qp = qpOASES::QProblem(nVariables, nConstraints);
	qp.setOptions(options);
	qp.init(H,g,A,0,0,lb,ub,nWSR,NULL,NULL,NULL,NULL,NULL,NULL);

	qp.getPrimalSolution(xOpt);
	//DEBUG
	//for (const auto& e : xOpt) {
	//    std::cout << e << std::endl;
	//}
	//

	Eigen::VectorXd decisionVariables(2*(N+M));

	for(int i=0;i<2*(N+M);++i){
		decisionVariables(i) = xOpt[i];
	}

	return decisionVariables;

}



void MPCSolver::logToFile() {
}




