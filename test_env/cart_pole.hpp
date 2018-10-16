#include <math.h>
#include <vector>
#include <iostream>
#include "abstract_env.hpp"

struct prm {
  double mCart;
  double mPend;
  double L;
} ;



class cartPole : public abstractEnv {

public:

	    prm pp;

	    cartPole(Eigen::VectorXd init_state,double dt,bool act_vis){
	    	this->num_state            = 4;

	    	this->state_bounds         = Eigen::MatrixXd(3,2);
			this->state_bounds         << -100,100,
				                          -100,100,
										  -M_PI,M_PI;
			const char *vinit[]        = {"x_c", "x_c_dot", "theta","theta_dot"};
			this->state_name           = std::vector<std::string>(vinit,vinit+4);
			this->init_state           = init_state;
			this->state                = this->init_state;
			this->dt                   = dt;
			this->active_visualization = act_vis;
			this->pp.mCart             = 0.1;
			this->pp.mPend             = 0.1;
			this->pp.L                 = 0.5;
			this->mes_acc              = Eigen::VectorXd(2);
	    };

	    Eigen::VectorXd Dynamics(Eigen::VectorXd cur_state,Eigen::VectorXd action, Eigen::VectorXd & mes_acc){
	    	double g   = 9.8;
	    	Eigen::VectorXd new_state(4);
	    	std::cout <<"action: " <<action << std::endl;
	    	std::cout << "state: " <<cur_state << std::endl;
			mes_acc(0) = (action(0) + pp.mPend*sin(cur_state(2))*(pp.L*pow(cur_state(3),2)-g*cos(cur_state(2))))/(pp.mCart+pp.mPend*pow(sin(cur_state(2)),2));
			//std::cout << mes_acc << std::endl;
			std::cout << "before crash" << std::endl;
         	mes_acc(1) = (-action(0)*cos(cur_state(2))- pp.mPend*pp.L*pow(cur_state(3),2)*sin(cur_state(2))*cos(cur_state(2)) + (pp.mPend+pp.mCart)*g*sin(cur_state(2)))/(pp.L*(pp.mCart+pp.mPend*pow(sin(cur_state(2)),2)));
         	std::cout << "after crash" << std::endl;
			new_state(0)= cur_state(1);
			new_state(1)= mes_acc(0);
			new_state(2)= cur_state(3);
			new_state(3)= mes_acc(1);

			return new_state;

	    }

	    Eigen::VectorXd Dynamics(Eigen::VectorXd cur_state,Eigen::VectorXd action){
			double g   = 9.8;
			Eigen::VectorXd new_state(4);
			Eigen::VectorXd cur_mes_acc(2);
			std::cout <<"action: " <<action << std::endl;
			std::cout << "state: " <<cur_state << std::endl;
			cur_mes_acc(0) = (action(0) + pp.mPend*sin(cur_state(2))*(pp.L*pow(cur_state(3),2)-g*cos(cur_state(2))))/(pp.mCart+pp.mPend*pow(sin(cur_state(2)),2));
			//std::cout << mes_acc << std::endl;
			std::cout << "before crash" << std::endl;
			cur_mes_acc(1) = (-action(0)*cos(cur_state(2))- pp.mPend*pp.L*pow(cur_state(3),2)*sin(cur_state(2))*cos(cur_state(2)) + (pp.mPend+pp.mCart)*g*sin(cur_state(2)))/(pp.L*(pp.mCart+pp.mPend*pow(sin(cur_state(2)),2)));
			std::cout << "after crash" << std::endl;
			new_state(0)= cur_state(1);
			new_state(1)= mes_acc(0);
			new_state(2)= cur_state(3);
			new_state(3)= mes_acc(1);

			return new_state;

	    }


	    void Wrapping(Eigen::VectorXd & state)
		{
			if (state(2)>M_PI)
				state(2) = -M_PI + (state(0)-M_PI);
			else if (state(2)<-M_PI)
				state(2) = M_PI - (-M_PI - state(0));
		};

		void Load_parameters(Eigen::VectorXd params){};
		void Render(){};
		void UpdateRender(Eigen::VectorXd state){};
        ~cartPole(){};

};

