#include "test_env/all_env.hpp"
#include "solvers/all_solvers.hpp"
#include "MPCregulator.hpp"
#include "MPCtracker.hpp"
#include <iostream>




int main(){

	std::string filename_env = "env_parameters.xml";
	std::string filename = "parameters.xml";
	// env selector
    std::string  switch_env("2RR");
    // solver selector
    std::string  switch_solver("qpoases");
    // problem selector
    std::string  switch_problem("tracker");
    // swtich behaviour
	bool visualization       = false;
    bool log                 = true;
    // construction of the environment
    std::cout << "creating environment" << std::endl;
    P_unique_env env;
    if(switch_env.compare("cart_pole") == 0){
    	env.reset(new cartPole(filename_env,visualization,log));
    }
    else if(switch_env.compare("2RR") == 0) {
    	env.reset(new twoRRobot(filename_env,visualization,log));
    }
    // plot information about the current environment
	env->plotInfoEnv();
	// constructing solver
	P_solv qp;
	if(switch_solver.compare("qpoases") == 0){
		qp.reset(new qpoasesSolver(filename));
	}else if(switch_solver.compare("") == 0){

	}
	qp->plotInfoQP();
	// constructing MPC problem
	P_unique_MPCinstance mpc;
	if(switch_problem.compare("regulator") == 0){
		mpc.reset(new MPCregulator(filename,qp));
	}else if(switch_problem.compare("tracker") == 0){
		bool online_comp = true;
		std::vector<traj> refs;
		refs.push_back(sin_f);                            // q1
		refs.push_back(cos_f);                            // q2
		refs.push_back(d_sin_f);                          // d_q1
		refs.push_back(d_cos_f);                          // d_q2
		param_vec param;
		param.push_back(std::vector<double>{M_PI/2,1.0});          // q1
		param.push_back(std::vector<double>{M_PI/3,1.0});          // q2
		param.push_back(std::vector<double>{M_PI/2,1.0,env->dt});  // d_q1
		param.push_back(std::vector<double>{M_PI/3,1.0,env->dt});  // d_q2
		trajectories traj = trajectories(env->dt,env->ft,qp->getPredictionDim(),refs,param,online_comp);
		mpc.reset(new MPCtracker(filename,qp,traj));

	}

	// simulation loop
	Eigen::VectorXd action(mpc->getControlDim());
	Eigen::VectorXd mes_acc(2);
	Eigen::VectorXd new_state(mpc->getStateDim());
	Eigen::VectorXd cur_state(mpc->getStateDim());
	// solvers initialization
	action = mpc->Init(env->init_state);
	if(env->feedback_lin){
		P_DynComp comp   = env->GetDynamicalComponents(cur_state);
		// the choice of cur_state.tail(2) is tailored for the 2R robot it has to be extended
		action         = comp->M*action + comp->S*cur_state.tail(2) + comp->g;
	}
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
		if(env->feedback_lin){
			P_DynComp comp   = env->GetDynamicalComponents(cur_state);
			// the choice of cur_state.tail(2) is tailored for the 2R robot it has to be extended
			action         = comp->M*action + comp->S*cur_state.tail(2) + comp->g;
		}
	}

	if(log){
		env->logToFile();
	}

	std::cout << "the end" << std::endl;

}
