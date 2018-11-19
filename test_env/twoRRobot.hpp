/*
 * twoRRobot.hpp
 *
 *  Created on: Nov 19, 2018
 *      Author: vale
 */

#ifndef TEST_ENV_TWORROBOT_HPP_
#define TEST_ENV_TWORROBOT_HPP_




#include <math.h>
#include <vector>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <sstream>
#include "abstract_env.hpp"

struct prm {
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

	    prm pp;

	    twoRRobot(const std::string filename,bool act_vis,bool log = false){
	    	// i create a string stream for concatenating strings
			std::stringstream ss;
			// i get the current working directory
			fs::path pathfs = fs::current_path();
			// convert to a string
			std::string path = pathfs.string();
			// concat string
			ss << path <<  "/current_functions/" << filename;
			// get the final path
			std::string full_path = ss.str();
			// Create empty property tree object
			pt::ptree tree;
			// Parse the XML into the property tree.
			pt::read_xml(full_path, tree);
	    	this->num_state            = 4;
	    	this->state_bounds         = Eigen::MatrixXd(3,2);
			this->state_bounds         << -100,100,
				                          -100,100,
										  -M_PI,M_PI;
			const char *vinit[]        = {"theta1", "theta2", "theta1_dot","theta2_dot"};
			this->state_name           = std::vector<std::string>(vinit,vinit+4);
			this->mes_acc              = Eigen::VectorXd(2);
            // particular care has to be taken for copying the init_state into an eigen vector
			// here i initialize the init_state variable
			this->init_state           = Eigen::VectorXd(this->num_state);
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

	    };

	    twoRRobot(Eigen::VectorXd init_state,double dt,double ft,prm param,bool act_vis){
	    	    this->num_state              = 4;
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
	   	};


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
        ~cartPole(){};

};




#endif /* TEST_ENV_TWORROBOT_HPP_ */
