/*
 * MPCtracker.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: vale
 */

#ifndef MPCTRACKER_HPP_
#define MPCTRACKER_HPP_

/*
 * MPCregulator.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: vale
 */



#include "MPCproblem.hpp"


class MPCtracker : public MPCproblem {

public:

	Eigen::VectorXd delta_action;
	Eigen::VectorXd inner_x;      // here i store full mpc tracker state [cur_state,cur_action,ref]
	int             current_step;

	MPCtracker(const std::string filename,P_solv  solv);
	Eigen::VectorXd Init(Eigen::VectorXd state_0_in);
	Eigen::VectorXd ComputeControl(Eigen::VectorXd state_i_in);
	~MPCtracker(){};

};





#endif /* MPCTRACKER_HPP_ */

