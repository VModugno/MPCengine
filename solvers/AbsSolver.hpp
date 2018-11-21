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

struct ProblemDetails{

	std::string      external_variables;  // we are optimizing problem parameters
	std::string      type;                // fixed or LTV
	std::string      problemClass;        // tracker or regulator (for now only two classes maybe i will need to extend it)
};

class AbsSolver {
public:

	// GET function
	int getStateDim(){return this->n;};
	int getControlDim(){return this->m;};
	int getOutputDim(){return this->p;};
	int getPredictionDim(){return this->N;};
    // virtual function
	// initialize the solver if necessary
	virtual Eigen::VectorXd initSolver(Eigen::VectorXd x0_in,Eigen::VectorXd  x0_ext,ProblemDetails & pd) = 0;
	virtual Eigen::VectorXd solveQP(Eigen::VectorXd xi_in,Eigen::VectorXd  x0_ext,ProblemDetails &  pd) = 0;

    virtual                 ~AbsSolver(){};

protected:
    // problem parameters
	int n;        // state   space dim
	int m;        // control space dim
	int p;        // output  space dim
	int N;        // prediction window
	int N_constr; // number of constraints

};

typedef std::unique_ptr<AbsSolver> P_unique_solv;
typedef std::shared_ptr<AbsSolver> P_solv;


#endif /* SOLVERS_MPCSOLVER_HPP_ */
