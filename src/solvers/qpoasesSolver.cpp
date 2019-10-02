#include "solvers/qpoasesSolver.hpp"
#include <sstream>
#include <stdlib.h>
#include <chrono>



/*qpoasesSolver::qpoasesSolver(int n,int m,int p,int N, int N_constr,std::string type,std::string solver,bool direct_solution){
	// Set up parameters
	this->n                     = n;
	this->m                     = m;
	this->p                     = p;
    this->N                     = N;
    this->N_constr              = N_constr;
    this->nVariables_batch      = this->N * this->m;
    this->nConstraints_batch    = this->N * this->N_constr;
    this->direct_solution       = direct_solution;
    if(direct_solution){
		this->qp   = qpOASES::QProblem(nVariables_batch,nConstraints_batch);
	}else{
		this->sqp  = qpOASES::SQProblem(nVariables_batch,nConstraints_batch);
	}
    original_nWSR  = 300;
	this->nWSR     = original_nWSR;
	// set qpoases option
	//options.setToReliable();
	options.setToMPC();
	options.printLevel           = qpOASES::PL_HIGH;//qpOASES::PL_HIGH;
	options.enableNZCTests       = qpOASES::BT_TRUE;
	options.enableFlippingBounds = qpOASES::BT_TRUE;

	time_perfomance = true;

}*/


void qpoasesSolver::initDim(pt::ptree & tree){

	this->type    = tree.get<std::string>("parameters.Entry.type");
	if(type.compare("statemachine")==0){
		// Set up parameters
		/*Eigen::VectorXd n_,m_,q_,N_constr_,state_machine_pattern;
		int number_of_models     = tree.get<int>("parameters.Entry.number_of_models");
		int N_                   = tree.get<int>("parameters.Entry.N");
		n_                       = Eigen::VectorXd(number_of_models);
		m_                       = Eigen::VectorXd(number_of_models);
		q_                       = Eigen::VectorXd(number_of_models);
		N_constr_                = Eigen::VectorXd(number_of_models);
		std::stringstream ss_n(tree.get<std::string>("parameters.Entry.n"));
		std::stringstream ss_m(tree.get<std::string>("parameters.Entry.m"));
		std::stringstream ss_q(tree.get<std::string>("parameters.Entry.q"));
		double cur_n,cur_m,cur_q;
		for (int i = 0; i<number_of_models; i++){
			ss_n >> cur_n;
			ss_m >> cur_m;
			ss_q >> cur_q;
			n_(i) = cur_n;
			m_(i) = cur_m;
			q_(i) = cur_q;
		}*/
		// here i get only the largest dimension because that is the only way to manage the statemachine properly (i hope)
		int number_of_models = tree.get<int>("parameters.Entry.number_of_models");
		std::stringstream ss_n(tree.get<std::string>("parameters.Entry.n"));
		std::stringstream ss_m(tree.get<std::string>("parameters.Entry.m"));
		double cur_n,cur_m,max_n = 0,max_m = 0;
		for (int i = 0; i<number_of_models; i++){
			ss_n >> cur_n;
			ss_m >> cur_m;
			if(cur_n > max_n)
				max_n = cur_n;
			if(cur_m > max_m)
				max_m = cur_m;
			//std::cout << max_m << std::endl;
		}
        this->n                   = max_n;
        this->m                   = max_m;
		this->N                   = tree.get<int>("parameters.Entry.N");
		this->dim_input_model     = Eigen::VectorXd(N);
		double cur_val;
		std::stringstream ss_dim_pattern(tree.get<std::string>("parameters.Entry.state_machine_control_dim_pattern"));
		for (int i = 0; i<N; i++){
			ss_dim_pattern >> cur_val;
			this->dim_input_model(i) = cur_val;

		}

		this->nVariables_batch    = tree.get<int>("parameters.Entry.nVariables_batch");
		this->nConstraints_batch  = tree.get<int>("parameters.Entry.nConstraints_batch");
	}
	else{
		// Set up parameters
		this->n                   = tree.get<int>("parameters.Entry.n");;
		this->m                   = tree.get<int>("parameters.Entry.m");
		this->q                   = tree.get<int>("parameters.Entry.q");
		this->N                   = tree.get<int>("parameters.Entry.N");
		this->N_constr            = tree.get<int>("parameters.Entry.N_constr");
		this->nVariables_batch    = this->N * this->m;
		this->nConstraints_batch  = this->N * this->N_constr;
	}

}
// in case of statemachine problem I'm not going to initialize the n m q variables because i do not use them
// anywhere inside the methods of QPOASES. here i only compute nVariables_batch and nConstraints_batch for now


qpoasesSolver::qpoasesSolver(const std::string filename,bool direct_solution){

	pt::ptree tree = ReadParameterXml(filename);

	// in order to manage the case of statemachine mpc i need to deal with the case
	// where the dimensions of the problem are arrays
	this->initDim(tree);
    // here i initialize the internal variable that contains the current optimal predicted solution
	this-> x_Opt   = Eigen::VectorXd(nVariables_batch);
    this->direct_solution     = direct_solution;
    if(direct_solution){
    	this->qp   = qpOASES::QProblem(nVariables_batch,nConstraints_batch);
    }else{
    	this->sqp  = qpOASES::SQProblem(nVariables_batch,nConstraints_batch);
    }
    original_nWSR  = 1000;
    this->nWSR     = original_nWSR;
    // set qpoases option
	options.setToReliable();
	//options.setToMPC();
	options.printLevel           = qpOASES::PL_NONE;//qpOASES::PL_HIGH; qpOASES::PL_NONE
	options.enableNZCTests       = qpOASES::BT_TRUE;
	options.enableFlippingBounds = qpOASES::BT_TRUE;

	time_perfomance = true;

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

	//int nVariables_batch   = this->N * this->m;
	//int nConstraints_batch = this->N * this->N_constr;
    // fitness matrices
	qpOASES::real_t H[nVariables_batch*nVariables_batch];
	qpOASES::real_t g[nVariables_batch];
	// constraints matrices
	qpOASES::real_t A[nConstraints_batch*nVariables_batch];
	qpOASES::real_t ubA[nConstraints_batch];
	// solution
	qpOASES::real_t xOpt[nVariables_batch];

	Eigen::VectorXd decisionVariables;
	int cur_dim;
	if(pd.type.compare("statemachine")==0){
		cur_dim = dim_input_model(pd.cur_index_pred_win);
		decisionVariables = Eigen::VectorXd(cur_dim);
	}
	else{
		decisionVariables = Eigen::VectorXd(this->m);
	}
    // compute matrix batch problem
	computeMatrix(H,g,A,ubA,x0,x0_e,pd);
	// solve optimization problem
	qpOASES::returnValue ret;
	if(direct_solution){
		this->qp.setOptions(options);
		ret = qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		this->GetVerySimpleStatus(ret,false);
		// get results
		qp.getPrimalSolution(xOpt);
	}else{
		this->sqp.setOptions(options);
		ret = sqp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		this->GetVerySimpleStatus(ret,false);
		// get results
		sqp.getPrimalSolution(xOpt);
	}

	// copy current solutions into internal variable
	for(int i=0;i<nVariables_batch;++i)
		this->x_Opt(i) = xOpt[i];

	if(pd.type.compare("statemachine")==0){
		for(int i=0;i<cur_dim;++i){
			decisionVariables(i) = xOpt[i];
		}
	}else{
		for(int i=0;i<this->m;++i){
			decisionVariables(i) = xOpt[i];
		}

	}

    //DEBUG
	//std::cout <<"decisionVariables = "<<decisionVariables << std::endl;

    return decisionVariables;
}


Eigen::VectorXd qpoasesSolver::initSolver(double * x0_in,double * x0_ext,ProblemDetails & pd)
{
	//int nVariables   = this->N * this->m;
	//int nConstraints = this->N * this->N_constr;
    // fitness matrices
	qpOASES::real_t H[nVariables_batch*nVariables_batch];
	qpOASES::real_t g[nVariables_batch];
	// constraints matrices
	qpOASES::real_t A[nConstraints_batch*nVariables_batch];
	qpOASES::real_t ubA[nConstraints_batch];
	// solution
	qpOASES::real_t xOpt[nVariables_batch];

	Eigen::VectorXd decisionVariables;
	int cur_dim;
	if(pd.type.compare("statemachine")==0){
		cur_dim = dim_input_model(pd.cur_index_pred_win);
		decisionVariables = Eigen::VectorXd(cur_dim);
	}
	else{
		decisionVariables = Eigen::VectorXd(this->m);
	}

    // compute components
	computeMatrix(H,g,A,ubA,x0_in,x0_ext,pd);
	// solve optimization problem
	qpOASES::returnValue ret;
	if(direct_solution){
		this->qp.setOptions(options);
		ret = qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		this->GetVerySimpleStatus(ret,false);
		// get results
		qp.getPrimalSolution(xOpt);
	}else{
		this->sqp.setOptions(options);
		ret = sqp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		this->GetVerySimpleStatus(ret,false);
		// get results
		sqp.getPrimalSolution(xOpt);
	}


	// copy current solutions into internal variable
    for(int i=0;i<nVariables_batch;++i)
    	this->x_Opt(i) = xOpt[i];

	if(pd.type.compare("statemachine")==0){
		for(int i=0;i<cur_dim;++i){
			decisionVariables(i) = xOpt[i];
		}
	}else{
		for(int i=0;i<this->m;++i){
			decisionVariables(i) = xOpt[i];
		}

	}


    return decisionVariables;
}

Eigen::VectorXd qpoasesSolver::solveQP(Eigen::VectorXd xi_in,Eigen::VectorXd  xi_ext,ProblemDetails & pd) {

	double *xi;
	xi = xi_in.data(); // pointing to the area of memory owned by the eigen vector
	double *xi_e;
	xi_e = xi_ext.data();

	//int nVariables_batch   = this->N*this->m;
	//int nConstraints_batch = this->N*this->N_constr;

	qpOASES::real_t H[nVariables_batch*nVariables_batch];
	qpOASES::real_t g[nVariables_batch];

	qpOASES::real_t A[nConstraints_batch*nVariables_batch];
	qpOASES::real_t lb[nConstraints_batch];
	qpOASES::real_t ubA[nConstraints_batch];

	qpOASES::real_t xOpt[nVariables_batch];

	Eigen::VectorXd decisionVariables;
	int cur_dim;
	if(pd.type.compare("statemachine")==0){
		//DEBUG
		//std::cout << "dim_input_model = "<< dim_input_model << std::endl;

		cur_dim = dim_input_model(pd.cur_index_pred_win);
		decisionVariables = Eigen::VectorXd(cur_dim);
	}
	else{
		decisionVariables = Eigen::VectorXd(this->m);
	}

	// DEBUGGING TIME
	std::chrono::high_resolution_clock::time_point start;
	std::chrono::high_resolution_clock::time_point stop;
	std::chrono::duration<double> duration;
	if(time_perfomance){
		start = std::chrono::high_resolution_clock::now();
	}
	// compute components
	computeMatrix(H,g,A,ubA,xi,xi_e,pd);
	// DEBUGGING TIME
	if(time_perfomance){
		stop = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::duration<double> >(stop - start);
		std::cout <<"compute matrices time = " <<duration.count()<<" s" << std::endl;
	}

	//DEBUG
	//std::cout << "this->nWSR = "<<this->nWSR<<std::endl;
    // restore nWSR
	// DEBUGGING TIME
	if(time_perfomance){
		start = std::chrono::high_resolution_clock::now();
	}
	// compute solutions
	qpOASES::returnValue ret;
	this->nWSR = original_nWSR;
	if(direct_solution){
		qp.reset();
		this->qp.setOptions(options);
		ret        = qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		qp.getPrimalSolution(xOpt);
	}else{
		ret        = sqp.hotstart(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		bool pass  = this->GetVerySimpleStatus(ret,false);
		if(!pass){
			//DEBUG
			//std::cout << "this->nWSR = "<<this->nWSR<<std::endl;
			// restore nWSR
			this->nWSR = original_nWSR;
			// resetting QP and restarting it
			sqp.reset();
			this->sqp.setOptions(options);
			sqp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		}
		// compute
		sqp.getPrimalSolution(xOpt);
	}
	//DEBUGGING TIME
	if(time_perfomance){
		stop = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::duration<double> >(stop - start);
		std::cout <<"compute qp time = " <<duration.count()<<" s" << std::endl;
	}

	// copy current solutions into internal variable
	for(int i=0;i<nVariables_batch;++i)
		this->x_Opt(i) = xOpt[i];

	// collecting first action
	if(pd.type.compare("statemachine")==0){
		for(int i=0;i<cur_dim;++i){
			decisionVariables(i) = xOpt[i];
		}
	}else{
		for(int i=0;i<this->m;++i){
			decisionVariables(i) = xOpt[i];
		}

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
	Eigen::VectorXd decisionVariables;
	int cur_dim;
	if(pd.type.compare("statemachine")==0){
		cur_dim = dim_input_model(pd.cur_index_pred_win);
		decisionVariables = Eigen::VectorXd(cur_dim);
	}
	else{
		decisionVariables = Eigen::VectorXd(this->m);
	}

	// DEBUGGING TIME
	std::chrono::high_resolution_clock::time_point start;
	std::chrono::high_resolution_clock::time_point stop;
	std::chrono::duration<double> duration;
	if(time_perfomance){
		start = std::chrono::high_resolution_clock::now();
	}
	// compute components
	computeMatrix(H,g,A,ubA,xi_in,xi_ext,pd);
	// DEBUGGING TIME
	if(time_perfomance){
		stop = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
		std::cout <<"compute matrices time = " <<duration.count() << " s" << std::endl;
	}
	// DEBUGGING TIME
	if(time_perfomance){
			start = std::chrono::high_resolution_clock::now();
	}
	// compute solutions
	this->nWSR = original_nWSR;
	qpOASES::returnValue ret;
	if(direct_solution){
		qp.reset();
		this->qp.setOptions(options);
		ret        = qp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		qp.getPrimalSolution(xOpt);
	}else{
		ret       = sqp.hotstart(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		bool pass = this->GetVerySimpleStatus(ret,false);
		if(!pass){
			this->nWSR = original_nWSR;
			// resetting QP and restarting it
			sqp.reset();
			this->sqp.setOptions(options);
			sqp.init(H,g,A,NULL,NULL,NULL,ubA,this->nWSR,NULL);
		}
		//DEBUG
		//std::cout << "this->nWSR" << this->nWSR << std::endl;
		// compute
		sqp.getPrimalSolution(xOpt);
	}
	//DEBUGGING TIME
	if(time_perfomance){
		stop = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast< std::chrono::duration<double> >(stop - start);
		std::cout <<"compute qp time = " <<duration.count()<< " s" << std::endl;
	}

	// copy current solutions into internal variable
	for(int i=0;i<nVariables_batch;++i)
		this->x_Opt(i) = xOpt[i];

	if(pd.type.compare("statemachine")==0){
		for(int i=0;i<cur_dim;++i){
			decisionVariables(i) = xOpt[i];
		}
	}else{
		for(int i=0;i<this->m;++i){
			decisionVariables(i) = xOpt[i];
		}

	}

	return decisionVariables;

}


void qpoasesSolver::plotInfoQP(){
	if(this->type .compare("statemachine")==0){
		std::cout << "max n = " << this->n << std::endl;
		std::cout << "max m = " << this->m << std::endl;
		std::cout << "N = " << this->N << std::endl;

	}else{
		std::cout << "n = " << this->n << std::endl;
		std::cout << "m = " << this->m << std::endl;
		std::cout << "q = " << this->q << std::endl;
		std::cout << "N = " << this->N << std::endl;
		std::cout << "N_constr = " << this->N_constr << std::endl;
	}

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






