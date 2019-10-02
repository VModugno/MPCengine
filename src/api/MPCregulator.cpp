/*
 * MPCregulator.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: vale
 */

#include "api/MPCregulator.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>


namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

MPCregulator::MPCregulator(const std::string filename,P_solv solv,P_oracle oracle)
{
	pt::ptree tree     = ReadParameterXml(filename);
	this->ex_var_dim   = tree.get<int>("parameters.Entry.external_dim"); // true or false
	// copy shared pointer
	this->solver       = solv;
    // initialize action
    this->action       = Eigen::VectorXd::Zero(solver->getControlDim());
    //[x_0,index]
    this->inner_x      = Eigen::VectorXd::Zero(solver->getStateDim() + 2);
    // initialize internal sample step
    this->inner_step         = 0;
    // initialize all sample counter
    this->current_step       = 0;
    // initialize current window iterator
    this->current_pred_win   = 1;
    // initialize sample control time
    this->ext_control_sample_time = tree.get<double>("parameters.Entry.ext_dt");
	this->mpc_sample_time         = tree.get<double>("parameters.Entry.internal_dt");
	this->relative_duration       = mpc_sample_time/ext_control_sample_time;
	this->trigger_update          = false;
    // initialize problem details
    this->pd.type                 = tree.get<std::string>("parameters.Entry.type");       // fixed or LTV
	this->pd.external_variables   = tree.get<std::string>("parameters.Entry.external_x"); // true or false
    // i initialize the external  variables if the current problem has set them
    if(this->pd.external_variables.compare("true") == 0){
		this->external_variables  = Eigen::VectorXd::Zero(this->ex_var_dim);
	}else{
	// even if external variables are not used we need to initialize the external_variables to zero with dim 2
	// (the explanation of dim 2 is in the matlab files)
		this->external_variables = Eigen::VectorXd::Zero(2);
	}
    this->oracle = oracle;
}

Eigen::VectorXd MPCregulator::Init(Eigen::VectorXd state_0_in){
	if(pd.type.compare("ltv")==0){
		Eigen::VectorXd trace = oracle->ComputePlan(state_0_in);
		this->inner_x << trace,inner_step,current_pred_win;
	}
	else if(pd.type.compare("fixed")==0 || pd.type.compare("statemachine")==0){
		//DEBUG
		std::cout <<"state_0_in.size() = "<<state_0_in.size() << std::endl;
		std::cout <<"this->inner_x.size() = "<<this->inner_x.size() << std::endl;

		this->inner_x << state_0_in,this->inner_step,current_pred_win;
	}
	this->pd.cur_index_pred_win = this->inner_step;
	this->action = solver->initSolver(this->inner_x,this->external_variables,this->pd);
	// update step
    this->current_step = this->current_step + 1;
    innerCounterUpdate();
	return this->action;
}

Eigen::VectorXd MPCregulator::ComputeControl(Eigen::VectorXd state_i_in){
	if(pd.type.compare("ltv")==0){
		Eigen::VectorXd trace = oracle->ComputePlan(state_i_in);
		this->inner_x << trace,inner_step,current_pred_win;
	}
	else if(pd.type.compare("fixed")==0 || pd.type.compare("statemachine")==0){
		this->inner_x << state_i_in,this->inner_step,current_pred_win;
	}
	this->pd.cur_index_pred_win = this->inner_step;
	this->action = solver->solveQP(this->inner_x,this->external_variables,this->pd);
	// update step
	this->current_step = this->current_step + 1;
	innerCounterUpdate();
	return this->action;
}



