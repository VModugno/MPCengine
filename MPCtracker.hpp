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
#include "trajectories.hpp"



class MPCtracker : public MPCproblem {

public:
	trajectories    traj;
	Eigen::VectorXd delta_action;
	Eigen::VectorXd inner_x;           // here I store full mpc tracker state [cur_state,cur_action,ref]
	Eigen::VectorXd ref;               // vector to store the current ref
	double          dt;                // time step;
	int             current_step;
	double          current_time_step;

	MPCtracker(const std::string filename,P_solv solv,trajectories & traj);
	Eigen::VectorXd Init(Eigen::VectorXd state_0_in);
	Eigen::VectorXd ComputeControl(Eigen::VectorXd state_i_in);
	~MPCtracker(){};

};





#endif /* MPCTRACKER_HPP_ */

