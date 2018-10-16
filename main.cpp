#include "test_env/cart_pole.hpp"
#include "MPCSolver.hpp"
#include<iostream>




int main(){
	// construction of the enviroment
	Eigen::VectorXd init_state(4);
	init_state     << 4,3,1,2;
	double dt      = 0.1;
	double steps   = 2000;
	std::cout << "creating environment" << std::endl;
	cartPole env   = cartPole(init_state,dt,false);

	// construction of the MPC controller
	int n        = 4;
    int m        = 1;
    int p        = 1;
    int N        = 3;
	int N_constr = 4;
	MPCSolver qp =MPCSolver(n,m,p,N,N_constr);
	// solvers initialization
	qp.initSolver(init_state);

	// simulation loop
	Eigen::VectorXd action(1);
	action(0) = 1;
	Eigen::VectorXd mes_acc(2);
	Eigen::VectorXd new_state(4);
	Eigen::VectorXd cur_state(4);

	cur_state = init_state;

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
