#include "solvers/qpoasesSolver.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <sstream>
#include <stdlib.h>
#include <chrono>


namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

qpoasesSolver::qpoasesSolver(int n,int m,int p,int N, int N_constr,std::string type,std::string solver){
	// Set up parameters
	this->n           = n;
	this->m           = m;
	this->p           = p;
    this->N           = N;
    this->N_constr    = N_constr;
    int nVariables    = this->N * this->m;
    int nConstraints  = this->N * this->N_constr;
    this->qp          = qpOASES::SQProblem(nVariables,nConstraints);
    this->nWSR        = 3000;
}

qpoasesSolver::qpoasesSolver(const std::string filename){
    // i create a string stream for concatenating strings
	std::stringstream ss;
    // i get the current working directory
	fs::path pathfs = fs::current_path();
	// convert to a string
	std::string path = pathfs.string();
	// concat string
	ss << path << "/configuration_file/" << filename;
	// get the final path
	std::string full_path = ss.str();

	//DEBUG
	std::cout << full_path << std::endl;


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
    int nVariables    = this->N * this->m;
    int nConstraints  = this->N * this->N_constr;
    this->qp          = qpOASES::SQProblem(nVariables,nConstraints);
    this->nWSR        = 5000000;
}


Eigen::VectorXd qpoasesSolver::initSolver(Eigen::VectorXd  x0_in,Eigen::VectorXd  x0_ext,ProblemDetails & pd)
{
	//DEBUG
	//std::cout << "x0_in = "<<x0_in<<std::endl;
	//std::cout << "x0_ext = "<<x0_ext<<std::endl;

	// eigen to array conversion
	double *x0;
	x0   = x0_in.data(); // pointing to the area of memory owned by the eigen vector
	double *x0_e;
	x0_e = x0_ext.data();

	//DEBUG
	//for(int ii = 0; ii<4; ii++)
	//	std::cout << x0[ii] << " ";
	//std::cout << std::endl;


	int nVariables_batch   = this->N * this->m;
	int nConstraints_batch = this->N * this->N_constr;
	// qpoases option
	qpOASES::Options options;
    //options.setToReliable();
	options.setToMPC();
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
    // compute matrix batch problem
	computeMatrix(H,g,A,ubA,x0,x0_e,pd);
	// solve optimization problem
	qpOASES::returnValue ret;
	ret = qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
	this->GetVerySimpleStatus(ret,true);
    // get results
	qp.getPrimalSolution(xOpt);

	for(int i=0;i<this->m;++i){
		decisionVariables(i) = xOpt[i];
	}

    //DEBUG
	//std::cout <<"decisionVariables = "<<decisionVariables << std::endl;

    return decisionVariables;
}


Eigen::VectorXd qpoasesSolver::initSolver(double * x0_in,double * x0_ext,ProblemDetails & pd)
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
	computeMatrix(H,g,A,ubA,x0_in,x0_ext,pd);
	// solve optimization problem
	qpOASES::returnValue ret;
	ret = qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
	this->GetVerySimpleStatus(ret,true);
    // get results
	qp.getPrimalSolution(xOpt);

	for(int i=0;i<this->m;++i){
		decisionVariables(i) = xOpt[i];
	}

    return decisionVariables;
}

Eigen::VectorXd qpoasesSolver::solveQP(Eigen::VectorXd xi_in,Eigen::VectorXd  xi_ext,ProblemDetails & pd) {

	double *xi;
	xi = xi_in.data(); // pointing to the area of memory owned by the eigen vector
	double *xi_e;
	xi_e = xi_ext.data();

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
	computeMatrix(H,g,A,ubA,xi,xi_e,pd);
	// compute solutions
	qpOASES::int_t new_nWSR = 30000;
	qpOASES::returnValue ret;

	//DEBUG
	//std::cout << "this->nWSR = "<<this->nWSR<<std::endl;
    // restore nWSR
	this->nWSR = 30000;
	ret        = qp.hotstart(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
	bool pass  = this->GetVerySimpleStatus(ret,true);
	if(!pass){
		//DEBUG
	    //std::cout << "this->nWSR = "<<this->nWSR<<std::endl;
		// restore nWSR
	    this->nWSR = 30000;
	    // resetting QP and restarting it
		qp.reset();
		qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
	}
    // compute
	qp.getPrimalSolution(xOpt);
	for(int i=0;i<m;++i){
		decisionVariables(i) = xOpt[i];
	}

	return decisionVariables;

}


Eigen::VectorXd qpoasesSolver::solveQP(double *xi_in,double *xi_ext,ProblemDetails & pd) {

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
	computeMatrix(H,g,A,ubA,xi_in,xi_ext,pd);
	// compute solutions
	//qpOASES::int_t new_nWSR = 30000;
	this->nWSR = 30000;
	qpOASES::returnValue ret;
	ret       = qp.hotstart(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
	bool pass = this->GetVerySimpleStatus(ret,true);
	if(!pass){
		this->nWSR = 30000;
		// resetting QP and restarting it
		qp.reset();
		qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
	}
	//std::cout << "this->nWSR" << this->nWSR << std::endl;
    // compute
	qp.getPrimalSolution(xOpt);
	for(int i=0;i<m;++i){
		decisionVariables(i) = xOpt[i];
	}

	return decisionVariables;

}


void qpoasesSolver::plotInfoQP(){
	std::cout << "n = " << this->n << std::endl;
	std::cout << "m = " << this->m << std::endl;
	std::cout << "p = " << this->p << std::endl;
	std::cout << "N = " << this->N << std::endl;
	std::cout << "N_constr = " << this->N_constr << std::endl;
}


bool qpoasesSolver::GetVerySimpleStatus(qpOASES::returnValue ret,bool debug = false){

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

void qpoasesSolver::computeMatrix(qpOASES::real_t H[],qpOASES::real_t g[],qpOASES::real_t A[],qpOASES::real_t ubA[],double * xi_in,double * xi_ext,ProblemDetails & pd){

	// add all the cases given the problem details (remove this line ASAP)
	//if(pd.type.compare("fixed") && pd.external_variables.compare("false")){
		compute_H(H,xi_in,xi_ext);
		compute_g(g,xi_in,xi_ext);
		compute_A(A,xi_in,xi_ext);
		compute_ub(ubA,xi_in,xi_ext);
	//}
}






