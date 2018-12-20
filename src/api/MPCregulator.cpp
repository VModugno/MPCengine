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

MPCregulator::MPCregulator(const std::string filename,P_solv solv)
{
	std::stringstream ss;
	// i get the current working directory
	fs::path pathfs = fs::current_path();
	// convert to a string
	std::string path = pathfs.string();
	// concat string
	ss << path << "/configuration_file/" << filename;
	// get the final path
	std::string full_path = ss.str();
	// Create empty property tree object
	pt::ptree tree;
	// Parse the XML into the property tree.
	pt::read_xml(full_path, tree);
	this->pd.type               = tree.get<std::string>("parameters.Entry.type");       // fixed or LTV
	this->pd.external_variables = tree.get<std::string>("parameters.Entry.external_x"); // true or false
	this->ex_var_dim            = tree.get<int>("parameters.Entry.external_dim"); // true or false
	// copy shared pointer
	this->solver       = solv;
    // initialize action
    this->action       = Eigen::VectorXd::Zero(solver->getControlDim());
    //[x_0,index]
    this->inner_x      = Eigen::VectorXd::Zero(solver->getStateDim() + 1);
    // initialize internal sample step
    this->inner_step   = 0;
    // i initialize the external  variables if the current problem has set them
    if(this->pd.external_variables.compare("true") == 0){
		this->external_variables = Eigen::VectorXd::Zero(this->ex_var_dim);
	}else{
	// even if external variables are not used we need to initialize the external_variables to zero with dim 2
	// (the explanation of dim 2 is in the matlab files)
		this->external_variables = Eigen::VectorXd::Zero(2);
	}

}

Eigen::VectorXd MPCregulator::Init(Eigen::VectorXd state_0_in){
	if(pd.type.compare("ltv")==0){
		Eigen::VectorXd trace = oracle->ComputePlan(state_0_in);
		this->inner_x << trace,inner_step;
	}else if(pd.type.compare("fixed")==0){
		this->inner_x << state_0_in,this->inner_step;
	}
	this->action = solver->initSolver(this->inner_x,this->external_variables,this->pd);
	// update step
    this->current_step = this->current_step + 1;
    innerStepUpdate();
	return this->action;
}

Eigen::VectorXd MPCregulator::ComputeControl(Eigen::VectorXd state_i_in){
	if(pd.type.compare("ltv")==0){
		Eigen::VectorXd trace = oracle->ComputePlan(state_i_in);
		this->inner_x << trace,inner_step;
	}else if(pd.type.compare("fixed")==0){
		this->inner_x << state_i_in,this->inner_step;
	}
	this->action = solver->solveQP(this->inner_x,this->external_variables,this->pd);
	// update step
	this->current_step = this->current_step + 1;
	innerStepUpdate();
	return this->action;
}



