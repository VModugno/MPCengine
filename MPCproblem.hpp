/*
 * MPCproblem.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: vale
 */

#ifndef MPCPROBLEM_HPP_
#define MPCPROBLEM_HPP_

#include "solvers/MPCsolver.hpp"

struct ProblemDetails{

	bool             external_variables;  // we are optimizing something else?
	std::string      type;                // fixed or LTV
	std::string      problemClass;        // tracker or regulator (for now only two classes maybe i will need to extend it)
};

class MPCproblem {
public:

	P_solv           solve;
	ProblemDetails   pd;
    // virtual function
	// initialize the solver if necessary
	virtual Eigen::VectorXd Init(Eigen::VectorXd x0_in) = 0;
	virtual Eigen::VectorXd ComputeControl(Eigen::VectorXd xi_in) = 0;

    virtual                 ~MPCproblem(){};

};

typedef std::unique_ptr<MPCproblem> P_unique_MPCinstance;
typedef std::shared_ptr<MPCproblem> P_MPCinstance;





#endif /* MPCPROBLEM_HPP_ */
