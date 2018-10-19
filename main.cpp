#include "test_env/cart_pole.hpp"
#include "MPCSolver.hpp"
#include<iostream>




int main(){
	// construction of the environment
	std::cout << "creating environment" << std::endl;
	std::string filename_env = "env_parameters.xml";
	cartPole env             = cartPole(filename_env,false);
	env.plotInfoEnv();
	// construction of the MPC controller
	std::string filename = "parameters.xml";
	MPCSolver qp         = MPCSolver(filename);
	qp.plotInfoQP();

	// simulation loop
	Eigen::VectorXd action(qp.getControlDim());
	Eigen::VectorXd mes_acc(2);
	Eigen::VectorXd new_state(qp.getStateDim());
	Eigen::VectorXd cur_state(qp.getStateDim());
	// solvers initialization
	action = qp.initSolver(env.init_state);
	// initializing variables for simulation
	cur_state = env.init_state;
    int steps = env.ft/env.dt;
	std::cout << "starting simulation" << std::endl;
	for(int i=0;i<steps;i++){
		// updating environment
		std::cout <<"action = "<<action << std::endl;
		env.Step(action,new_state,mes_acc);
		std::cout <<"new_state = "<<new_state << std::endl;
		// update state
		cur_state = new_state;
		// compute control actions
		action = qp.solveQP(cur_state);
	}

	std::cout << "the end" << std::endl;

}
