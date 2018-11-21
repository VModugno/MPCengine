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


class MPCsolver {
public:


    // virtual function
	// initialize the solver if necessary
	virtual Eigen::VectorXd initSolver(Eigen::VectorXd x0_in) = 0;
	virtual Eigen::VectorXd solveQP(Eigen::VectorXd xi_in) = 0;

    virtual                 ~MPCsolver(){};

};

typedef std::unique_ptr<MPCsolver> P_unique_solv;
typedef std::shared_ptr<MPCsolver> P_solv;


#endif /* SOLVERS_MPCSOLVER_HPP_ */
