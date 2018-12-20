/*
 * MPCregulator.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: vale
 */

#ifndef MPCREGULATOR_HPP_
#define MPCREGULATOR_HPP_

#include "MPCproblem.hpp"


class MPCregulator : public MPCproblem {

public:

	int             current_step;

	MPCregulator(const std::string filename,P_solv  solv,P_oracle oracle = NULL);
	Eigen::VectorXd Init(Eigen::VectorXd state_0_in);
	Eigen::VectorXd ComputeControl(Eigen::VectorXd state_i_in);
	~MPCregulator(){};

};





#endif /* MPCREGULATOR_HPP_ */
