#include "test_env/cart_pole.hpp"
#include<iostream>




int main(){
	// construction of the enviroment
	Eigen::VectorXd init_state(4);
	init_state     << 0,0,0,0;
	double dt      = 0.1;
	double steps   = 2000;
	std::cout << "creating environment" << std::endl;
	cartPole env   = cartPole(init_state,dt,false);

	// construction of the MPC controller


	// simulation loop
	Eigen::VectorXd action(1);
	action(0) = 1;
	Eigen::VectorXd mes_acc(2);
	Eigen::VectorXd new_state(4);

	std::cout << "starting simulation" << std::endl;
	for(int i=0;i<steps;i++){
		env.Step(action,mes_acc,new_state);
		std::cout << new_state << std::endl;
	}

}
