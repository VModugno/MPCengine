/*
 * MPCproblem.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: vale
 */

#ifndef MPCPROBLEM_HPP_
#define MPCPROBLEM_HPP_

#include "solvers/AbsSolver.hpp"


class MPCproblem {
public:

	P_solv           solver;               // all the dimension of the problem are stored in the abstract solver
	ProblemDetails   pd;
	Eigen::VectorXd  external_variables;   // here i store the external variables for the current experiment
	int              ex_var_dim;           // dimension of the exeternal variables vector (0 if there is no external variable vector)
	Eigen::VectorXd  action;               // here i define the action vectors


    void SetExtVariables(Eigen::VectorXd cur_ext_var){
    	this->external_variables = cur_ext_var;
    };
    // virtual function
	// initialize the solver if necessary
	virtual Eigen::VectorXd Init(Eigen::VectorXd state_0_in) = 0;
	virtual Eigen::VectorXd ComputeControl(Eigen::VectorXd state_i_in) = 0;

    virtual                 ~MPCproblem(){};

};

typedef std::unique_ptr<MPCproblem> P_unique_MPCinstance;
typedef std::shared_ptr<MPCproblem> P_MPCinstance;





#endif /* MPCPROBLEM_HPP_ */
