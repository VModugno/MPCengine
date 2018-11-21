#include "test_env/all_env.hpp"
#include "solvers/all_solvers.hpp"
#include "MPCregulator.hpp"
#include "MPCtracker.hpp"
#include <iostream>




int main(){
	// construction of the environment
	std::cout << "creating environment" << std::endl;
	std::string filename_env = "env_parameters.xml";
	bool visualization       = false;
    bool log                 = true;
    P_unique_env env;
    // environment selector
    std::string  switch_env("2RR");
    if(switch_env.compare("cart_pole") == 0){
    	env.reset(new cartPole(filename_env,visualization,log));
    }
    else if(switch_env.compare("2RR") == 0) {
    	env.reset(new twoRRobot(filename_env,visualization,log));
    }
    // plot information about the current environment
	env->plotInfoEnv();
	// construction of the MPC controller
	std::string filename = "parameters.xml";
	qpoasesSolver qp     = qpoasesSolver(filename);
	qp.plotInfoQP();

	P_unique_MPCinstance mpc;
	// MPC problem selector
	std::string  switch_env("2RR");
	if(switch_env.compare("cart_pole") == 0){
		mpc.reset(new cartPole(filename,qp));
	}
	//else if(switch_env.compare("2RR") == 0) {
	//	mpc.reset(new twoRRobot(filename_env,visualization,log));
	//}



	// simulation loop
	Eigen::VectorXd action(qp.getControlDim());
	Eigen::VectorXd mes_acc(2);
	Eigen::VectorXd new_state(qp.getStateDim());
	Eigen::VectorXd cur_state(qp.getStateDim());
	// solvers initialization
	action = qp.initSolver(env->init_state);
	// initializing variables for simulation
	cur_state = env->init_state;
    int steps = env->ft/env->dt;
	std::cout << "starting simulation" << std::endl;
	for(int i=0;i<steps;i++){
		// updating environment
		std::cout <<"action = "<<action << std::endl;
		env->Step(action,new_state,mes_acc);
		std::cout <<"new_state = "<<new_state << std::endl;
		// update state
		cur_state = new_state;
		// compute control actions
		action = qp.solveQP(cur_state);
		// computed torque for 2r robot
		if(switch_env.compare("2RR")){
			DynComp comp   = env->GetDynamicalComponents(cur_state);
			// the choice of cur_state.tail(2) is tailored for the 2R robot it has to be extended
			action         = comp.M*action + comp.S*cur_state.tail(2) + comp.g;
		}
	}

	if(log){
		env->logToFile();
	}

	std::cout << "the end" << std::endl;

}
