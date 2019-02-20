/*
 * twoRRobot.hpp
 *
 *  Created on: Nov 19, 2018
 *      Author: vale
 */

#ifndef TEST_ENV_TWORROBOT_HPP_
#define TEST_ENV_TWORROBOT_HPP_

#include "abstract_env.hpp"


struct prmRR {
  double l1;
  double l2;
  double m1;
  double m2;
  double c1x;
  double c1y;
  double c1z;
  double c2x;
  double c2y;
  double c2z;
  double J1zz;
  double J2zz;
} ;

namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

class twoRRobot : public abstractEnv {

public:

	    prmRR pp;

	    twoRRobot(const std::string filename,bool act_vis,bool log = false){
	    	// i create a string stream for concatenating strings
			std::stringstream ss;
			// i get the current working directory
			fs::path pathfs = fs::current_path();
			// convert to a string
			std::string path = pathfs.string();
			// concat string
			ss << path <<  "/configuration_file/" << filename;
			// get the final path
			std::string full_path = ss.str();
			// Create empty property tree object
			pt::ptree tree;
			// Parse the XML into the property tree.
			pt::read_xml(full_path, tree);
	    	this->dim_state            = 4;
	    	this->DOF                  = 2;
	    	this->state_bounds         = Eigen::MatrixXd(4,2);
			this->state_bounds         << -2*M_PI,2*M_PI,
					                      -2*M_PI,2*M_PI,
						                  -2*M_PI,2*M_PI,
					                      -2*M_PI,2*M_PI;
			const char *vinit[]        = {"theta1", "theta2", "theta1_dot","theta2_dot"};
			this->state_name           = std::vector<std::string>(vinit,vinit+4);
			this->mes_acc              = Eigen::VectorXd(2);
            // particular care has to be taken for copying the init_state into an eigen vector
			// here i initialize the init_state variable
			this->init_state           = Eigen::VectorXd(this->dim_state);
			std::stringstream sss(tree.get<std::string>("parameters.Entry.init_state"));
			double d;
			int    count=0;
			    while (sss >> d)
			    {
			    	this->init_state(count) = d;
			        count++;
			    }
			this->state                  = this->init_state;
			this->dt                     = tree.get<double>("parameters.Entry.delta");
			this->ft                     = tree.get<double>("parameters.Entry.ft");
			this->pp.l1                  = tree.get<double>("parameters.Entry.l1");
			this->pp.l2                  = tree.get<double>("parameters.Entry.l2");
			this->pp.m1                  = tree.get<double>("parameters.Entry.m1");
			this->pp.m2                  = tree.get<double>("parameters.Entry.m2");
			this->pp.c1x                 = tree.get<double>("parameters.Entry.c1x");
			this->pp.c1y                 = tree.get<double>("parameters.Entry.c1y");
			this->pp.c1z                 = tree.get<double>("parameters.Entry.c1z");
			this->pp.c2x                 = tree.get<double>("parameters.Entry.c2x");
			this->pp.c2y                 = tree.get<double>("parameters.Entry.c2y");
			this->pp.c2z                 = tree.get<double>("parameters.Entry.c2z");
			this->pp.J1zz                = tree.get<double>("parameters.Entry.J1zz");
		    this->pp.J2zz                = tree.get<double>("parameters.Entry.J2zz");

			this->active_visualization   = act_vis;
			this->log                    = log;
			this->comps.reset(new DynComp(DOF));
			this->feedback_lin           = true;

	    };

	    twoRRobot(Eigen::VectorXd init_state,double dt,double ft,prmRR param,bool act_vis){
			this->dim_state              = 4;
			this->DOF                    = 2;
			this->state_bounds           = Eigen::MatrixXd(4,2);
			this->state_bounds           <<-2*M_PI,2*M_PI,
										  -2*M_PI,2*M_PI,
										  -2*M_PI,2*M_PI,
										  -2*M_PI,2*M_PI;
			const char *vinit[]          = {"x_c", "x_c_dot", "theta","theta_dot"};
			this->state_name             = std::vector<std::string>(vinit,vinit+4);
			this->init_state             = init_state;
			this->state                  = this->init_state;
			this->dt                     = dt;
			this->active_visualization   = act_vis;
			this->ft                     = ft;
			this->pp.l1                  = param.l1;
			this->pp.l2                  = param.l2;
			this->pp.m1                  = param.m1;
			this->pp.m2                  = param.m2;
			this->pp.c1x                 = param.c1x;
			this->pp.c1y                 = param.c1y;
			this->pp.c1z                 = param.c1z;
			this->pp.c2x                 = param.c2x;
			this->pp.c2y                 = param.c2y;
			this->pp.c2z                 = param.c2z;
			this->pp.J1zz                = param.J1zz;
			this->pp.J2zz                = param.J2zz;
			this->mes_acc                = Eigen::VectorXd(2);
			this->comps.reset(new DynComp(DOF));
			this->feedback_lin           = true;
	   	};



	    Eigen::MatrixXd get_M(Eigen::VectorXd state,prmRR pp){
	    	Eigen::MatrixXd M(2,2);
	    	         //prm.J1zz+ prm.J2zz+ prm.l1^2*prm.m1    + prm.l1^2*prm.m2    + prm.l2^2*prm.m2 + 2*prm.c1x*prm.l1*prm.m1 + 2*prm.c2x*prm.l2*prm.m2 + 2*prm.c2x*prm.l1*prm.m2*cos(state(2)) + 2*prm.l1*prm.l2*prm.m2*cos(state(2)) - 2*prm.c2y*prm.l1*prm.m2*sin(state(2));
			M(0,0) = pp.J1zz + pp.J2zz + pow(pp.l1,2)*pp.m1 + pow(pp.l1,2)*pp.m2 + pow(pp.l2,2)*pp.m2 + 2*pp.c1x*pp.l1*pp.m1 + 2*pp.c2x*pp.l2*pp.m2 + 2*pp.c2x*pp.l1*pp.m2*cos(state(1)) + 2*pp.l1*pp.l2*pp.m2*cos(state(1)) - 2*pp.c2y*pp.l1*pp.m2*sin(state(1));
			M(0,1) = pp.J2zz + pow(pp.l2,2)*pp.m2 + 2*pp.c2x*pp.l2*pp.m2 + pp.c2x*pp.l1*pp.m2*cos(state(1)) + pp.l1*pp.l2*pp.m2*cos(state(1)) - pp.c2y*pp.l1*pp.m2*sin(state(1));
			M(1,0) = M(0,1);
			M(1,1) = pp.m2*pow(pp.l2,2) + 2*pp.c2x*pp.m2*pp.l2 + pp.J2zz;
			return M;
	    }

	    Eigen::MatrixXd get_S(Eigen::VectorXd state,prmRR pp){
	    	Eigen::MatrixXd S(2,2);
			S(0,0) = -state(3)*pp.l1*pp.m2*(pp.c2y*cos(state(1)) + pp.c2x*sin(state(1)) + pp.l2*sin(state(1)));
			S(0,1) = -pp.l1*pp.m2*(state(2) + state(3))*(pp.c2y*cos(state(1)) + pp.c2x*sin(state(1)) + pp.l2*sin(state(1)));
			S(1,0) = state(2)*pp.l1*pp.m2*(pp.c2y*cos(state(1)) + pp.c2x*sin(state(1)) + pp.l2*sin(state(1)));
			S(1,1) = 0;
			return S;

	    }

	    Eigen::VectorXd get_C(Eigen::VectorXd state,prmRR pp){
	    	Eigen::VectorXd C(2);
			C(0) = -state(3)*pp.l1*pp.m2*(2*state(2) + state(3))*(pp.c2y*cos(state(1)) + pp.c2x*sin(state(1)) + pp.l2*sin(state(1)));
			C(1) = pow(state(2),2)*pp.l1*pp.m2*(pp.c2y*cos(state(1)) + pp.c2x*sin(state(1)) + pp.l2*sin(state(1)));
			return C;
	    }

	    Eigen::VectorXd get_g(Eigen::VectorXd state,prmRR pp){
	    	Eigen::VectorXd g(2);
			double g0 = 9.80665;
			g(0) = g0*pp.m1*(pp.c1x*cos(state(0)) + pp.l1*cos(state(0)) - pp.c1y*sin(state(0))) + g0*pp.m2*(pp.c2x*cos(state(0) + state(1)) + pp.l2*cos(state(0) + state(1)) - pp.c2y*sin(state(0) + state(1)) + pp.l1*cos(state(0)));
			g(1) = g0*pp.m2*(pp.c2x*cos(state(0) + state(1)) + pp.l2*cos(state(0) + state(1)) - pp.c2y*sin(state(0) + state(1)));
			return g;
	    }

	    Eigen::VectorXd ReshapeAction(Eigen::VectorXd action){
	    	    	return action;
	    }

	    P_DynComp GetDynamicalComponents(Eigen::VectorXd cur_state)
	    {

	    	this->comps->M = get_M(cur_state,this->pp);
	    	this->comps->C = get_C(cur_state,this->pp);
	    	this->comps->g = get_g(cur_state,this->pp);

			return this->comps;

	    }

	    Eigen::VectorXd Dynamics(Eigen::VectorXd cur_state,Eigen::VectorXd action, Eigen::VectorXd & mes_acc){

	    	Eigen::VectorXd new_state(4);

	    	this->comps->M = get_M(cur_state,this->pp);
	    	this->comps->C = get_C(cur_state,this->pp);
	    	this->comps->g = get_g(cur_state,this->pp);

	    	Eigen::MatrixXd M_inv(2,2);
	    	M_inv = comps->M.inverse();

	    	mes_acc = M_inv*(action - comps->C - comps->g);


			new_state(0) = cur_state(2);
			new_state(1) = cur_state(3);
			new_state(2) = mes_acc(0);
			new_state(3) = mes_acc(1);

			return new_state;

	    };

	    Eigen::VectorXd Dynamics(Eigen::VectorXd cur_state,Eigen::VectorXd action){

	    	Eigen::VectorXd new_state(4),mes_acc(2);

	    	this->comps->M = get_M(cur_state,this->pp);
	    	this->comps->C = get_C(cur_state,this->pp);
	    	this->comps->g = get_g(cur_state,this->pp);

			//Eigen::MatrixXd M_inv = M.inverse();

			mes_acc = comps->M.inverse()*(action - comps->C - comps->g);


			new_state(0) = cur_state(2);
			new_state(1) = cur_state(3);
			new_state(2) = mes_acc(0);
			new_state(3) = mes_acc(1);

			return new_state;

	    };


	    void Wrapping(Eigen::VectorXd & state)
		{

		};

	    void plotInfoEnv(){
	    	std::cout << "init_state = " << this->init_state << std::endl;
			std::cout << "dt = "         << this->dt         << std::endl;
			std::cout << "ft = "         << this->ft         << std::endl;
			std::cout << "l1 = "         << this->pp.l1      << std::endl;
			std::cout << "l2 = "         << this->pp.l2      << std::endl;
			std::cout << "m1 = "         << this->pp.m1      << std::endl;
			std::cout << "m2 = "         << this->pp.m2      << std::endl;
			std::cout << "c1x = "        << this->pp.c1x     << std::endl;
			std::cout << "c1y = "        << this->pp.c1y     << std::endl;
			std::cout << "c1z = "        << this->pp.c1z     << std::endl;
			std::cout << "c2x = "        << this->pp.c2x     << std::endl;
			std::cout << "c2y = "        << this->pp.c2y     << std::endl;
			std::cout << "c2z = "        << this->pp.c2z     << std::endl;
			std::cout << "J1zz = "       << this->pp.J1zz    << std::endl;
			std::cout << "J2zz = "       << this->pp.J2zz    << std::endl;
	    };


		void Load_parameters(Eigen::VectorXd params){};
		void Render(){};
		void UpdateRender(Eigen::VectorXd state){};
        ~twoRRobot(){};

};




#endif /* TEST_ENV_TWORROBOT_HPP_ */
