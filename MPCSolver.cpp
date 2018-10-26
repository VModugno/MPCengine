#include "MPCSolver.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <sstream>
#include <stdlib.h>
#include <chrono>


namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

MPCSolver::MPCSolver(int n,int m,int p,int N, int N_constr,std::string type,std::string solver){
	// Set up parameters
	this->n           = n;
	this->m           = m;
	this->p           = p;
    this->N           = N;
    this->N_constr    = N_constr;
    int nVariables    = this->N * this->m;
    int nConstraints  = this->N * this->N_constr;
    this->type        = type;
    this->solver      = solver;
    this->qp          = qpOASES::SQProblem(nVariables,nConstraints);
    this->nWSR        = 3000;
}

MPCSolver::MPCSolver(const std::string filename){
    // i create a string stream for concatenating strings
	std::stringstream ss;
    // i get the current working directory
	fs::path pathfs = fs::current_path();
	// convert to a string
	std::string path = pathfs.string();
	// concat string
	ss << path <<  "/current_functions/" << filename;
	// get the final path
	std::string full_path = ss.str();


	// Create empty property tree object
	pt::ptree tree;
	// Parse the XML into the property tree.
	pt::read_xml(full_path, tree);
	// Set up parameters
	this->n           = tree.get<int>("parameters.Entry.n");
	this->m           = tree.get<int>("parameters.Entry.m");
	this->p           = tree.get<int>("parameters.Entry.q");
    this->N           = tree.get<int>("parameters.Entry.N");
    this->N_constr    = tree.get<int>("parameters.Entry.N_constr");
    this->type        = tree.get<std::string>("parameters.Entry.type");
    this->solver      = tree.get<std::string>("parameters.Entry.solver");
    int nVariables    = this->N * this->m;
    int nConstraints  = this->N * this->N_constr;
    this->qp          = qpOASES::SQProblem(nVariables,nConstraints);
    this->nWSR        = 30000;
}


Eigen::VectorXd MPCSolver::initSolver(Eigen::VectorXd  x0_in)
{
	// eigen to array conversion
	double *x0;
	x0 = x0_in.data(); // pointing to the area of memory owned by the eigen vector

	//DEBUG
	for(int ii = 0; ii<4; ii++)
		std::cout << x0[ii] << " ";
	std::cout << std::endl;


	int nVariables_batch   = this->N * this->m;
	int nConstraints_batch = this->N * this->N_constr;
	// qpoases option
	qpOASES::Options options;
    options.setToReliable();
    options.printLevel           = qpOASES::PL_HIGH;
    options.enableNZCTests       = qpOASES::BT_TRUE;
    options.enableFlippingBounds = qpOASES::BT_TRUE;
    this->qp.setOptions(options);
    // fitness matrices
	qpOASES::real_t H[nVariables_batch*nVariables_batch];
	qpOASES::real_t g[nVariables_batch];
	// constraints matrices
	qpOASES::real_t A[nConstraints_batch*nVariables_batch];
	qpOASES::real_t ubA[nConstraints_batch];
	// solution
	qpOASES::real_t xOpt[nVariables_batch];
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


Eigen::VectorXd MPCSolver::initSolver(double * x0)
{
	int nVariables   = this->N * this->m;
	int nConstraints = this->N * this->N_constr;
	// qpoases option
	qpOASES::Options options;
    options.setToMPC();
    options.printLevel=qpOASES::PL_HIGH;
    this->qp.setOptions(options);
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

Eigen::VectorXd MPCSolver::solveQP(Eigen::VectorXd xi_in) {

	double *xi;
	xi = xi_in.data(); // pointing to the area of memory owned by the eigen vector

	int nVariables_batch   = this->N*this->m;
	int nConstraints_batch = this->N*this->N_constr;

	qpOASES::real_t H[nVariables_batch*nVariables_batch];
	qpOASES::real_t g[nVariables_batch];

	qpOASES::real_t A[nConstraints_batch*nVariables_batch];
	qpOASES::real_t lb[nConstraints_batch];
	qpOASES::real_t ubA[nConstraints_batch];

	qpOASES::real_t xOpt[nVariables_batch];
	Eigen::VectorXd decisionVariables(this->m);

	// compute components
	compute_H(H);
	compute_g(g,xi);
	compute_A(A);
	compute_ub(ubA,xi);
	// compute solutions
	//qp.reset();
	qpOASES::int_t new_nWSR = 30000;
	qp.hotstart(H,g,A,NULL,NULL,NULL,ubA,new_nWSR,NULL);
	//qp.init(H,g,A,NULL,NULL,NULL,ubA,new_nWSR,NULL);
    // compute
	//std::cout << "this->nWSR " << this->nWSR << std::endl;
	qp.getPrimalSolution(xOpt);
	for(int i=0;i<m;++i){
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
	qp.reset();
	//qp.hotstart(g,NULL,NULL,NULL,ubA,this->nWSR,NULL);
	qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
	std::cout << "this->nWSR" << this->nWSR << std::endl;
    // compute
	qp.getPrimalSolution(xOpt);
	for(int i=0;i<m;++i){
		decisionVariables(i) = xOpt[i];
	}

	return decisionVariables;

}


void MPCSolver::plotInfoQP(){
	std::cout << "n = " << this->n << std::endl;
	std::cout << "m = " << this->m << std::endl;
	std::cout << "p = " << this->p << std::endl;
	std::cout << "N = " << this->N << std::endl;
	std::cout << "N_constr = " << this->N_constr << std::endl;
}

void MPCSolver::logToFile(std::string data2log) {

}




