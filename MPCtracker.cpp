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

MPCtracker::MPCtracker(const std::string filename,P_solv solv,trajectories & traj)
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

	// copy shared pointer for the solver class
    //this->solver.reset(solv.get());
	this->solver = solv;
    // copy trajectories
    this->traj   = traj;
    // initialize action
    this->action = Eigen::VectorXd::Zero(solver->getControlDim());
    // initialize delta action
    //DEBUG
    //std::cout<<"solver->getStateDim() = "<<solver->getStateDim() << ",solver->getControlDim() = "<< solver->getControlDim() << ",solver->getPredictionDim()*solver->getOutputDim()= "<<solver->getPredictionDim()*solver->getOutputDim()<<std::endl;
    this->delta_action = Eigen::VectorXd::Zero(solver->getControlDim());
    this->inner_x      = Eigen::VectorXd::Zero(solver->getStateDim() + solver->getPredictionDim()*solver->getOutputDim());
    this->ref          = Eigen::VectorXd::Zero(solver->getPredictionDim()*solver->getOutputDim());
    // initialize time step
    this->dt                 =traj.GetDt();
    // initialize internal sample step
    this->current_step       = 0;
    // initilize internal time step
    this->current_time_step  = 0;
    // i initialize the external  variables if the current problem has set them
    if(this->pd.external_variables.compare("true") == 0){
    	this->external_variables = Eigen::VectorXd::Zero(this->ex_var_dim);
    }else{
    // even if external variables are not used we need to initialize the external_variables to zero with dim 2
    // (the explanation of dim 2 is in the matlab files)
    	this->external_variables = Eigen::VectorXd::Zero(2);
    }
}

Eigen::VectorXd MPCtracker::Init(Eigen::VectorXd state_0_in){
	//DEBUG
	//std::cout << "ref.size() = "<< ref.size() << std::endl;

	ref = traj.ComputeTraj(current_time_step,current_step);
	//DEBUG
	//std::cout << "after computeTraj ref.size() = "<< ref.size()   << std::endl;
	//std::cout << "state_0_in.size() = "<< state_0_in.size()       << std::endl;
	//std::cout << "this->action.size() = "<< this->action.size()   << std::endl;
	//std::cout << "this->inner_x.size() = "<< this->inner_x.size() << std::endl;

	// DONT CHANGE! the right order is set inside the code generator class in matlab with the inner_x variables
	this->inner_x << state_0_in,this->action,ref;

	//DEBUG
	//std::cout << "this->inner_x = "<<this->inner_x<<std::endl;

	this->delta_action = solver->initSolver(this->inner_x,this->external_variables,this->pd);
    // update of last_action
	this->action  = this->action + this->delta_action;

	//DEBUG
	//std::cout<<"delta_action = "<< this->delta_action<<std::endl;
	//std::cout<<"action = "<<std::endl;

    // update step
	this->current_step = this->current_step + 1;
	// update time step
    this->current_time_step = this->current_time_step + this->dt;

    return this->action;
}

Eigen::VectorXd MPCtracker::ComputeControl(Eigen::VectorXd state_i_in){
	ref = traj.ComputeTraj(current_time_step,current_step);
	this->inner_x << state_i_in,this->action,ref;
	this->delta_action = solver->solveQP(this->inner_x,this->external_variables,this->pd);
	// update of last_action
    this->action  = this->action + this->delta_action;
    // update step
    this->current_step = this->current_step + 1;
    // update time step
    this->current_time_step = this->current_time_step + this->dt;

	return this->action;
}





