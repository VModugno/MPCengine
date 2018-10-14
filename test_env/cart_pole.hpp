#include <math.h>
#include "abstract_env.hpp"

struct prm {
  double mCart;
  double mPend;
  double L;
} ;



class cart_pole : public abstract_env {

public:
	    prm pp;


	    cart_pole(Eigen::VectorXd init_state,double dt,bool act_vis){
	    	this->num_state            = 4;
			this->state_bounds         << -100,100,
				                          -100,100,
										 -M_PI,M_PI;
			const char *vinit[]        = {"x_c", "x_c_dot", "theta","theta_dot"};
			this->state_name(vinit, end(vinit));
			this->init_state           = init_state;
			this->state                = this->init_state;
			this->dt                   = dt;
			this->active_visualization = act_vis;
			this->pp.mCart             =;
			this->pp.mPend             =;
			this->pp.L                 =;


	    }

	    Eigen::VectorXd Dynamics(Eigen::VectorXd action, Eigen::VectorXd & mes_acc){
	    	double g   = 9.8;
	    	Eigen::VectorXd & new_state;
			mes_acc(0) = (action(0) + pp.mPend*sin(state(2))*(pp.L*pow(state(3),2)-g*cos(state(2))))/(pp.mCart+pp.mPend*pow(sin(state(2)),2));
         	mes_acc(1) = (-action(0)*cos(state(2)) - pp.mPend*pp.L*pow(state(3),2)*sin(state(2))*cos(state(2)) + (pp.mPend+pp.mCart)*g*sin(state(2)))/(pp.L*(pp.mCart+pp.mPend*pow(sin(state(2)),2)));

			new_state  = [state(1); ...
						 mes_acc(0);...
						 state(3);...
						 mes_acc(1)];

	    }

	    Eigen::VectorXd Wrapping(Eigen::VectorXd state)
		{
			if (state(2)>M_PI)
				state(2) = -M_PI + (state(0)-M_PI);
			else if (state(2)<-M_PI)
				state(2) = M_PI - (-M_PI - state(0));


		};

		void Load_parameters(Eigen::VectorXd params){};
		void Render(){};
		void UpdateRender(Eigen::VectorXd state){};

};
