/*
 * 2Dxy_lip.hpp
 *
 *  Created on: Nov 29, 2018
 *      Author: vale
 */

#ifndef TEST_ENV_2DXY_LIP_HPP_
#define TEST_ENV_2DXY_LIP_HPP_


#include "abstract_env.hpp"

struct prmXYLip {
  double mCart;
  double mPend;
  double L;
} ;

namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

class TwoDxyLip : public abstractEnv {

public:

	    prmXYLip pp;

	    TwoDxyLip(const std::string filename,bool act_vis,bool log = false){
	    	// i create a string stream for concatenating strings
			std::stringstream ss;
			// i get the current working directory
			fs::path pathfs = fs::current_path();
			// convert to a string
			std::string path = pathfs.string();
			// concat string
			ss << path <<  "/solvers/current_functions/" << filename;
			// get the final path
			std::string full_path = ss.str();
			// Create empty property tree object
			pt::ptree tree;
			// Parse the XML into the property tree.
			pt::read_xml(full_path, tree);
	    	this->dim_state            = 4;
	    	this->DOF                  = 2;
	    	this->state_bounds         = Eigen::MatrixXd(4,2);
			this->state_bounds         << -100,100,
				                          -100,100,
										  -M_PI,M_PI,
										  -M_PI,M_PI;
			const char *vinit[]        = {"x_c", "x_c_dot", "theta","theta_dot"};
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
			this->state                = this->init_state;
			this->dt                   = tree.get<double>("parameters.Entry.delta");
			this->ft                   = tree.get<double>("parameters.Entry.ft");
			this->pp.mCart             = tree.get<double>("parameters.Entry.mCart");
			this->pp.mPend             = tree.get<double>("parameters.Entry.mPend");
			this->pp.L                 = tree.get<double>("parameters.Entry.L");
			this->active_visualization = act_vis;
			this->log                  = log;
			this->feedback_lin         = false;

	    };

	    TwoDxyLip(Eigen::VectorXd init_state,double dt,double ft,prmCP param,bool act_vis){
	    	    this->dim_state            = 4;
	    	    this->DOF                  = 2;
	   	    	this->state_bounds         = Eigen::MatrixXd(4,2);
	   			this->state_bounds         << -100,100,
	   				                          -100,100,
	   										  -M_PI,M_PI,
	   			                              -M_PI,M_PI;
	   			const char *vinit[]        = {"x_c", "x_c_dot", "theta","theta_dot"};
	   			this->state_name           = std::vector<std::string>(vinit,vinit+4);
	   			this->init_state           = init_state;
	   			this->state                = this->init_state;
	   			this->dt                   = dt;
	   			this->ft                   = ft;
	   			this->active_visualization = act_vis;
	   			this->pp.mCart             = param.mCart;
	   			this->pp.mPend             = param.mPend;
	   			this->pp.L                 = param.L;
	   			this->mes_acc              = Eigen::VectorXd(2);
	   			this->feedback_lin         = false;
	   	};


	    P_DynComp GetDynamicalComponents(Eigen::VectorXd cur_state){};

	    Eigen::VectorXd Dynamics(Eigen::VectorXd cur_state,Eigen::VectorXd action, Eigen::VectorXd & mes_acc){




	    };

	    Eigen::VectorXd Dynamics(Eigen::VectorXd cur_state,Eigen::VectorXd action){


	    };


	    void Wrapping(Eigen::VectorXd & state)
		{

		};

	    void plotInfoEnv(){
	    	std::cout << "init_state = " << this->init_state << std::endl;
			std::cout << "dt = "         << this->dt         << std::endl;
			std::cout << "ft = "         << this->ft         << std::endl;
			std::cout << "mCart = "      << this->pp.mCart   << std::endl;
			std::cout << "mPend = "      << this->pp.mPend   << std::endl;
			std::cout << "L = "          << this->pp.L       << std::endl;

	    };


		void Load_parameters(Eigen::VectorXd params){};
		void Render(){};
		void UpdateRender(Eigen::VectorXd state){};
        ~cartPole(){};

};



#endif /* TEST_ENV_2DXY_LIP_HPP_ */
