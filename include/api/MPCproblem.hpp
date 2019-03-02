/*
 * MPCproblem.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: vale
 */

#ifndef MPCPROBLEM_HPP_
#define MPCPROBLEM_HPP_

#include "solvers/AbsSolver.hpp"
#include "OraclePlanner.hpp"


class MPCproblem {
public:

	P_solv           solver;               // all the dimension of the problem are stored in the abstract solver
	P_oracle         oracle;               // i will use p oracle only if the problem is ltv
	ProblemDetails   pd;
	Eigen::VectorXd  inner_x;              // here I store full mpc tracker state
	Eigen::VectorXd  external_variables;   // here i store the external variables for the current experiment
	int              ex_var_dim;           // dimension of the external variables vector (0 if there is no external variable vector)
	Eigen::VectorXd  action;               // here i define the action vectors
	int              inner_step;           // this counter specify at which point of the prediction window we are
	int              current_pred_win;     // this iterator tells us in which prediction window we are (TODO example here to explain)

	// GET function
	int getStateDim()                    {return solver->getStateDim();};
	int getControlDim()                  {return solver->getControlDim();};
	int getOutputDim()                   {return solver->getOutputDim();};
	int getPredictionDim()               {return solver->getPredictionDim();};
	Eigen::VectorXd getPredictedOptSol() {return solver->getPredictedOptSol();};
	Eigen::VectorXd getDimInputModel()   {return solver->getDimInputModel();};
	int getInnerStep()      {return inner_step;};
	int getCurrentPredWin() {return current_pred_win;};
    void SetExtVariables(Eigen::VectorXd cur_ext_var){
    	this->external_variables = cur_ext_var;
    };
    // this function update inner step and reset it to zero when the new prediction window is reached
    void innerCounterUpdate(){
    	inner_step++;
    	if(inner_step > (this->getPredictionDim() - 1)){
    		inner_step       = 0;
    		current_pred_win = current_pred_win + 1;
    	}

    }

    // virtual function
	// initialize the solver if necessary
	virtual Eigen::VectorXd Init(Eigen::VectorXd state_0_in) = 0;
	virtual Eigen::VectorXd ComputeControl(Eigen::VectorXd state_i_in) = 0;
    virtual                 ~MPCproblem(){};

};

typedef std::unique_ptr<MPCproblem> P_unique_MPCinstance;
typedef std::shared_ptr<MPCproblem> P_MPCinstance;





#endif /* MPCPROBLEM_HPP_ */
