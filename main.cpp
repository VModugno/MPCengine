#include "test_env/cart_pole.hpp"
#include "MPCSolver.hpp"
#include<iostream>




int main(){
	// construction of the environment
	std::cout << "creating environment" << std::endl;
	std::string filename_env = "env_parameters.xml";
	cartPole env   = cartPole(filename_env,false);
	env.plotInfoEnv();
	// construction of the MPC controller
	std::string filename = "parameters.xml";
	MPCSolver qp         = MPCSolver(filename);
	qp.plotInfoQP();
	// solvers initialization
	qp.initSolver(env.init_state);

	// simulation loop
	Eigen::VectorXd action(qp.getControlDim());
	action(0) = 1;
	Eigen::VectorXd mes_acc(2);
	Eigen::VectorXd new_state(qp.getStateDim());
	Eigen::VectorXd cur_state(qp.getStateDim());
	// initializing variables for simulation
	cur_state = env.init_state;
    int steps = env.ft/env.dt;
	std::cout << "starting simulation" << std::endl;
	for(int i=0;i<steps;i++){
        // compute control actions
		action = qp.solveQP(cur_state);
		// updating environment
		env.Step(action,mes_acc,new_state);
		// update state
		cur_state = new_state;
		std::cout << new_state << std::endl;
	}

	std::cout << "the end" << std::endl;

}
