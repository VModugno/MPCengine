/*
 * MPCregulator.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: vale
 */

#include "MPCregulator.hpp"
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
	ss << path << "/solvers/current_functions/" << filename;
	// get the final path
	std::string full_path = ss.str();
	// Create empty property tree object
	pt::ptree tree;
	// Parse the XML into the property tree.
	pt::read_xml(full_path, tree);
	this->pd.type               = tree.get<std::string>("parameters.Entry.type");       // fixed or LTV
	this->pd.external_variables = tree.get<std::string>("parameters.Entry.external_x"); // true or false
	this->pd.problemClass       = "regulator";
	this->ex_var_dim            = tree.get<int>("parameters.Entry.ext_var_dim"); // true or false

	// copy shared pointer
    this->solver.reset(solv.get());
    // initialize action
    this->action(solver->getControlDim());
    // i initialize the external  variables if the current problem has set them
    if(this->pd.external_variables.compare("true") == 0){
    	this->external_variables(this->ex_var_dim);
    }
}

Eigen::VectorXd MPCregulator::Init(Eigen::VectorXd state_0_in){
	this->action = solver->initSolver(state_0_in,this->external_variables,this->pd);
	return this->action;
}

Eigen::VectorXd MPCregulator::ComputeControl(Eigen::VectorXd state_i_in){
	this->action = solver->solveQP(state_i_in,this->external_variables,this->pd);
	return this->action;
}



