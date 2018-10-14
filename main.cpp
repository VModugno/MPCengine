#include "test_env/cart_pole.hpp"
#include <Eigen/core>




int main(){
	Eigen::VectorXd init_state << 0,
			                      0,
								  0,
								  0;
	double dt = 0.1;
	cart_pole env = cart_pole(init_state,dt,false);
}
