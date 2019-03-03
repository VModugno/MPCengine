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
#include "trajectories/trajectories.hpp"

// here the state dim represents the extended state (original state plus control) due to the extension

class MPCtracker : public MPCproblem {

public:
	trajectories    traj;
	Eigen::VectorXd delta_action;
	Eigen::VectorXd ref;               // vector to store the current ref
	// TODO duplicate with internal_dt in mpcproblem; did it! to check
	//double          dt;                // time step;
	double          current_time_step;

	MPCtracker(const std::string filename,P_solv solv,trajectories & traj,P_oracle oracle = NULL);
	Eigen::VectorXd Init(Eigen::VectorXd state_0_in);
	Eigen::VectorXd ComputeControl(Eigen::VectorXd state_i_in);
	~MPCtracker(){};

};





#endif /* MPCTRACKER_HPP_ */

