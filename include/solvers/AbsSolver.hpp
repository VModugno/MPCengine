/*
 * AbstractSolvers.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: vale
 */

#ifndef SOLVERS_MPCSOLVER_HPP_
#define SOLVERS_MPCSOLVER_HPP_

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

namespace pt = boost::property_tree;

struct ProblemDetails{

	std::string      external_variables;  // we are optimizing problem parameters
	std::string      type;                // fixed  LTV or statemachine
	int              cur_index_pred_win;  // for now i use this variables to tell the solver the current sample inside the prediction windows
	//Eigen::VectorXd  dim_input_model;   // this vector contains the dimension of the current model given the current time sample in the control window
};

class AbsSolver {
public:

	// GET function
	int getStateDim(){return this->n;};
	int getControlDim(){return this->m;};
	int getOutputDim(){return this->q;};
	int getPredictionDim(){return this->N;};
	Eigen::VectorXd getPredictedOptSol(){return this->x_Opt;};
	Eigen::VectorXd getDimInputModel(){
		if(type.compare("statemachine")==0){
			return this->dim_input_model;
		}
		else{
			std::cout << "[WARNING]= dim_input_model is not used for this type( "<<this->type <<" )of mpc problem"<<std::endl;
			return Eigen::VectorXd::Zero(N);
		}
	};

    // virtual function
	// initialize the solver if necessary
	virtual void            initDim(pt::ptree & tree) = 0;
	virtual Eigen::VectorXd initSolver(Eigen::VectorXd x0_in,Eigen::VectorXd  x0_ext,ProblemDetails & pd) = 0;
	virtual Eigen::VectorXd solveQP(Eigen::VectorXd xi_in,Eigen::VectorXd  x0_ext,ProblemDetails &  pd) = 0;
    virtual void            plotInfoQP()=0;
    virtual                 ~AbsSolver(){};

protected:
    // problem parameters
	int n;                           // state   space dim
	int m;                           // control space dim
	int q;                           // output  space dim
	int N;                           // prediction window
	int N_constr;                    // number of constraints
	bool time_perfomance = false;    // activate or deactivate performance computation
	Eigen::VectorXd dim_input_model; // this vector contains the dimension of the current model given the current time sample in the control window
	Eigen::VectorXd x_Opt;           // this the vector with the current complete predicted solution;
	std::string type;                // type of mpc problem
};

typedef std::unique_ptr<AbsSolver> P_unique_solv;
typedef std::shared_ptr<AbsSolver> P_solv;


#endif /* SOLVERS_MPCSOLVER_HPP_ */
