/*
 * MPCtracker.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: vale
 */


/*
 * MPCregulator.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: vale
 */

#include "MPCtracker.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>


namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

MPCtracker::MPCtracker(const std::string filename,P_solv solv)
{
	std::stringstream ss;
	// i get the current working directory
	fs::path pathfs = fs::current_path();
	// convert to a string
	std::string path = pathfs.string();
	// concat string
	ss << path << "/solvers/current_functions/" << filename;
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
    this->solver.reset(solv.get());
    // initialize action
    this->action = Eigen::VectorXd::Zero(solver->getControlDim());
    // initialize delta action
    this->delta_action = Eigen::VectorXd::Zero(solver->getControlDim());
    this->inner_x      = Eigen::VectorXd::Zero(solver->getStateDim() + solver->getControlDim() + solver->getPredictionDim()*solver->getOutputDim());
    // initialize internal sample step
    this->current_step = 0;
    // i initialize the external  variables if the current problem has set them
    if(this->pd.external_variables.compare("true") == 0){
    	this->external_variables(this->ex_var_dim);
    }
}

Eigen::VectorXd MPCtracker::Init(Eigen::VectorXd state_0_in){
	Eigen::VectorXd ref;
	this->inner_x << state_0_in,this->action,ref;
	this->delta_action = solver->initSolver(this->inner_x,this->external_variables,this->pd);
    // update of last_action
	this->action  = this->action + this->delta_action;
    // update step
	this->current_step = this->current_step + 1;

	return this->action;
}

Eigen::VectorXd MPCtracker::ComputeControl(Eigen::VectorXd state_i_in){
	Eigen::VectorXd ref;
	this->inner_x << state_i_in,this->action,ref;
	this->delta_action = solver->solveQP(this->inner_x,this->external_variables,this->pd);
	// update of last_action
    this->action  = this->action + this->delta_action;
    // update step
    this->current_step = this->current_step + 1;

	return this->action;
}





