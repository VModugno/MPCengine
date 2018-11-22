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
    // selecting environment
    P_unique_env env;
    std::string  switch_env("2RR");
    if(switch_env.compare("cart_pole") == 0){
    	env.reset(new cartPole(filename_env,visualization,log));
    }
    else if(switch_env.compare("2RR") == 0) {
    	env.reset(new twoRRobot(filename_env,visualization,log));
    }
    // plot information about the current environment
	env->plotInfoEnv();
	// selecting sovler
	std::string filename = "parameters.xml";
	std::string  switch_solver("qpoases");
	P_solv qp;
	if(switch_solver.compare("qpoases") == 0){
		qp.reset(new qpoasesSolver(filename));
	}else if(switch_solver.compare("") == 0){

	}
	qp->plotInfoQP();
	// selecting MPC problem
	P_unique_MPCinstance mpc;
	std::string  switch_problem("regulator");
	if(switch_problem.compare("regulator") == 0){
		mpc.reset(new MPCregulator(filename,qp));
	}else if(switch_problem.compare("tracker") == 0){

	}

	// simulation loop
	Eigen::VectorXd action(mpc->getControlDim());
	Eigen::VectorXd mes_acc(2);
	Eigen::VectorXd new_state(mpc->getStateDim());
	Eigen::VectorXd cur_state(mpc->getStateDim());
	// solvers initialization
	action = mpc->Init(env->init_state);
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
		action = mpc->ComputeControl(cur_state);
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
