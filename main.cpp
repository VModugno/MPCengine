#include "test_env/cart_pole.hpp"
#include<iostream>




int main(){
	Eigen::VectorXd init_state(4);
	init_state << 0,0,0,0;
	double dt = 0.1;
	std::cout << "creating environment" << std::endl;
	cartPole env = cartPole(init_state,dt,false);
	std::cout << "created environment" << std::endl;

}
