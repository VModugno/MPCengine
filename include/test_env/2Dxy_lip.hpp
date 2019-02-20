/*
 * 2Dxy_lip.hpp
 *
 *  Created on: Nov 29, 2018
 *      Author: vale
 */

#ifndef TEST_ENV_2DXY_LIP_HPP_
#define TEST_ENV_2DXY_LIP_HPP_


#include "abstract_env.hpp"
#include <cmath>

struct prmXYLip {
	double h;
	double footSize_x;
	double footSize_y;
	// dummy states (references)
	double foot_to_foot_x;
	double foot_to_foot_y;
	double vref_x;
	double vref_y;
} ;

namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

class TwoDxyLip : public abstractEnv {

public:

	    Eigen::MatrixXd A; // dynamics matrix
	    Eigen::MatrixXd B; // input dynamic matrix
	    Eigen::MatrixXd C; // measure matrix
	    int dim_contr;     // control dimension
		int dim_measu;     // measure dimension
	    prmXYLip pp;

	    TwoDxyLip(const std::string filename,bool act_vis,bool log = false){
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
	    	this->dim_state            = 18;
	    	this->DOF                  = 6;
	    	// TODO update bounds
	    	this->state_bounds         = Eigen::MatrixXd(4,2);
			this->state_bounds         << -100,100,
				                          -100,100,
										  -M_PI,M_PI,
										  -M_PI,M_PI;
			const char *vinit[]        = {"x_com","x_com_dot","x_zmp","x_left_f_pos","x_left_f_pos_dot","x_right_f_pos","x_right_f_pos_dot","x_ref","x_v_com_ref",
	                                      "y_com","y_com_dot","y_zmp","y_left_f_pos","y_left_f_pos_dot","y_right_f_pos","y_right_f_pos_dot","y_ref","y_v_com_ref"};
			this->state_name           = std::vector<std::string>(vinit,vinit+18);
			// in this system mes_acc is not used
			this->mes_acc              = Eigen::VectorXd(6);
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
			this->pp.h                 = tree.get<double>("parameters.Entry.h");
			this->pp.footSize_x        = tree.get<double>("parameters.Entry.footSize_x");
			this->pp.footSize_y        = tree.get<double>("parameters.Entry.footSize_y");
			//this->pp.vref_x            = tree.get<double>("parameters.Entry.vref_x");
		    //this->pp.vref_y            = tree.get<double>("parameters.Entry.vref_y");
			this->active_visualization = act_vis;
			this->log                  = log;
			this->feedback_lin         = false;
			this->already_discretized  = true;
			this->dim_contr            = 6;
			this->dim_measu            = 12;

			// creating the system
            A = Eigen::MatrixXd::Zero(dim_state,dim_state);
            B = Eigen::MatrixXd::Zero(dim_state,dim_contr);
            C = Eigen::MatrixXd::Zero(dim_measu,dim_state);

            computeSystemMatrices(A,B,C);

	    };

	    TwoDxyLip(Eigen::VectorXd init_state,double dt,double ft,prmXYLip param,bool act_vis){
	    	    this->dim_state            = 18;
	    	    this->DOF                  = 6;
	    	    // to update bounds
	   	    	this->state_bounds         = Eigen::MatrixXd(4,2);
	   			this->state_bounds         << -100,100,
	   				                          -100,100,
	   										  -M_PI,M_PI,
	   			                              -M_PI,M_PI;
	   			const char *vinit[]        = {"x_com","x_com_dot","x_zmp","x_left_f_pos","x_left_f_pos_dot","x_right_f_pos","x_right_f_pos_dot","x_ref","x_v_com_ref",
                                              "y_com","y_com_dot","y_zmp","y_left_f_pos","y_left_f_pos_dot","y_right_f_pos","y_right_f_pos_dot","y_ref","y_v_com_ref"};;
	   			this->state_name           = std::vector<std::string>(vinit,vinit+18);
	   			this->init_state           = init_state;
	   			this->state                = this->init_state;
	   			this->dt                   = dt;
	   			this->ft                   = ft;
	   			this->active_visualization = act_vis;
	   			this->pp.h                 = param.h;
				this->pp.footSize_x        = param.footSize_x;
				this->pp.footSize_y        = param.footSize_y;
	   			this->mes_acc              = Eigen::VectorXd(2);
	   			this->feedback_lin         = false;
	   			this->already_discretized  = true;
	   			this->dim_contr            = 6;
	   		    this->dim_measu            = 12;
	   	};

	    Eigen::VectorXd ReshapeAction(Eigen::VectorXd action){
	    	    	return action;
	    }

	    P_DynComp GetDynamicalComponents(Eigen::VectorXd cur_state){};

	    Eigen::VectorXd Dynamics(Eigen::VectorXd cur_state,Eigen::VectorXd action, Eigen::VectorXd & mes_acc){
	    	// empty
	    };

	    Eigen::VectorXd Dynamics(Eigen::VectorXd cur_state,Eigen::VectorXd action){
            //DEBUG
	    	//std::cout << "A = "        <<A        <<std::endl;
	    	//std::cout << "B = "        <<B        <<std::endl;
	    	//std::cout << "cur_state = "<<cur_state<<std::endl;
	    	//std::cout << "action = "   << action  <<std::endl;

	    	Eigen::VectorXd new_state(this->dim_state);
	    	new_state = A*cur_state + B*action;
	    	return new_state;
	    };


	    void Wrapping(Eigen::VectorXd & state)
		{

		};

	    void plotInfoEnv(){
	    	std::cout << "init_state = " << this->init_state    << std::endl;
			std::cout << "dt = "         << this->dt            << std::endl;
			std::cout << "ft = "         << this->ft            << std::endl;
			std::cout << "h = "          << this->pp.h          << std::endl;
			std::cout << "footSize_x = " << this->pp.footSize_x << std::endl;
			std::cout << "footSize_y = " << this->pp.footSize_y << std::endl;

	    };

		void Load_parameters(Eigen::VectorXd params){};
		void Render(){};
		void UpdateRender(Eigen::VectorXd state){};
        ~TwoDxyLip(){};
private:
        void computeSystemMatrices(Eigen::MatrixXd& _A, Eigen::MatrixXd& _B, Eigen::MatrixXd& _C)
       	{
       	      // LIP model
       	      Eigen::Matrix3d A_lip;
       	      Eigen::Vector3d B_lip;
              double omega = sqrt(9.8/pp.h);

       	      double ch = cosh(omega*dt);
       	      double sh = sinh(omega*dt);
       	      A_lip << ch, sh/omega, 1-ch, omega*sh, ch, -omega*sh, 0, 0, 1;
       	      B_lip << dt-sh/omega, 1-ch, dt;

       	      // Foot model
       	      Eigen::Matrix2d A_foot_cont;
       	      A_foot_cont << 0, 1, 0, 0;
       	      Eigen::MatrixXd A_foot = Eigen::MatrixXd::Identity(2,2) + dt * A_foot_cont;
       	      Eigen::MatrixXd B_foot = dt * Eigen::Vector2d(0, 1);

       	      // Dummy states for foot-to-foot distance and reference velocity
       	      Eigen::MatrixXd A_dummy = Eigen::MatrixXd::Ones(1,1);
       	      // Full system
       	      Eigen::MatrixXd B_block = Eigen::MatrixXd::Zero(dim_state/2, dim_state/2);
       	      Eigen::VectorXd zero_l = Eigen::VectorXd::Zero(3);
       	      B_block = blkdiag(std::vector<Eigen::MatrixXd>( {B_lip, B_foot, B_foot} ));
       	      B_block.conservativeResize(B_block.rows()+2,Eigen::NoChange);
       	      B_block.row(B_block.rows()-2) = zero_l;
       	      B_block.row(B_block.rows()-1) = zero_l;
       	      //DEBUG
       	      //Eigen::MatrixXd test   = Eigen::MatrixXd::Zero(dim_state/2, dim_state/2);
       	      //test = blkdiag(std::vector<Eigen::MatrixXd>( {B_lip, B_foot, B_foot} ));
       	      //test.conservativeResize(test.rows()+2,Eigen::NoChange);
       	      //test.row(test.rows()-2) = zero_l;
       	      //test.row(test.rows()-1) = zero_l;
       	      //std::cout <<" test = "<< test << std::endl;
       	      std::cout <<"B_block = " <<B_block << std::endl;

       	      _A = blkdiag(std::vector<Eigen::MatrixXd>( {A_lip, A_foot, A_foot, A_dummy, A_dummy,
       	    					                          A_lip, A_foot, A_foot, A_dummy, A_dummy} ));
       	      _B = blkdiag(std::vector<Eigen::MatrixXd>( {B_block, B_block} ));

       	      Eigen::MatrixXd C_block = Eigen::MatrixXd::Zero(dim_measu/2, dim_state/2);

       	      C_block << 0, 1, 0, 0, 0, 0, 0, 0,-1,  // com velocity to vref
       	    	         0, 0, 0, 0, 1, 0, 0, 0, 0,  // left foot velocity
       	    	         0, 0, 0, 0, 0, 0, 1, 0, 0,  // left foot velocity
       	    	         0, 0, 1,-1, 0, 0, 0, 0, 0,  // zmp from left foot
       	    	         0, 0, 1, 0, 0,-1, 0, 0, 0,  // zmp from right foot
       	    	         0, 0, 0, 1, 0,-1, 0, 1, 0;  // foot to foot


       	      _C = blkdiag(std::vector<Eigen::MatrixXd>( {C_block, C_block} ));
       	 }

         Eigen::MatrixXd blkdiag(std::vector<Eigen::MatrixXd> matrices)
         {
            // Compute total matrix size
            int rows = 0, cols = 0;
            for(int i = 0; i < matrices.size(); i++)
            {
              rows += matrices[i].rows();
              cols += matrices[i].cols();
            }

            // Initialize matrix and fill blocks
            Eigen::MatrixXd result = Eigen::MatrixXd::Zero(rows, cols);

            int reachedRow = 0, reachedCol = 0;
            for(int i = 0; i < matrices.size(); i++)
            {
              result.block(reachedRow, reachedCol, matrices[i].rows(), matrices[i].cols()) = matrices[i];
              reachedRow += matrices[i].rows();
              reachedCol += matrices[i].cols();
            }

            return result;
         }

};



#endif /* TEST_ENV_2DXY_LIP_HPP_ */
