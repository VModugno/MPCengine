/*
 * MPCproblem.hpp
 *
 *  Created on: Nov 20, 2018
 *      Author: vale
 */

#ifndef MPCPROBLEM_HPP_
#define MPCPROBLEM_HPP_

#include "solvers/AbsSolver.hpp"
#include "OraclePlanner.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>


namespace pt = boost::property_tree;
namespace fs = boost::filesystem;


class MPCproblem {
public:

	P_solv           solver;               // all the dimension of the problem are stored in the abstract solver
	P_oracle         oracle;               // i will use p oracle only if the problem is ltv
	ProblemDetails   pd;
	Eigen::VectorXd  inner_x;              // here I store full mpc tracker state
	Eigen::VectorXd  external_variables;   // here i store the external variables for the current experiment
	int              ex_var_dim;           // dimension of the external variables vector (0 if there is no external variable vector)
	Eigen::VectorXd  action;               // here i define the action vectors
	double           mpc_sample_time;          //
	double           ext_control_sample_time;  //
	double           relative_duration;
	int              inner_step;           // this counter specify at which point of the prediction window we are
	int              current_pred_win;     // this iterator tells us in which prediction window we are (TODO example here to explain)
	bool             trigger_update;       // this is true when the controller sample coincide with the mpc sample (always true when mpc and controller are synched))
	int              current_step;         // this is a counter that count the real steps (if mpc is syncroinzed with controller current_step = inner_step )
	// GET function
	int getStateDim()                    {return solver->getStateDim();};
	int getControlDim()                  {return solver->getControlDim();};
	int getOutputDim()                   {return solver->getOutputDim();};
	int getPredictionDim()               {return solver->getPredictionDim();};
	Eigen::VectorXd getPredictedOptSol() {return solver->getPredictedOptSol();};
	Eigen::VectorXd getDimInputModel()   {return solver->getDimInputModel();};
	bool getTriggerUpdate()  {return trigger_update;};
	int  getInnerStep()      {return inner_step;};
	int  getCurrentPredWin() {return current_pred_win;};
    void SetExtVariables(Eigen::VectorXd cur_ext_var){
    	this->external_variables = cur_ext_var;
    };
    // this function update inner step and reset it to zero when the new prediction window is reached (TODO to explain better)
    void innerCounterUpdate(){

    	if(this->trigger_update){
    		this->trigger_update = false;
    	}
        // i update the internal counter only if it is the right time to do that
    	// his condition does not happens always when the controller is faster than the mpc
    	if(fmod(this->current_step,relative_duration)==0){
    		trigger_update = true;
			inner_step++;
			if(inner_step > (this->getPredictionDim() - 1)){
				inner_step       = 0;
				current_pred_win = current_pred_win + 1;
			}
    	}

    }

    pt::ptree ReadParameterXml(std::string filename){
    	std::stringstream ss;
		// i get the current working directory
		fs::path pathfs = fs::current_path();
		// convert to a string
		std::string path = pathfs.string();
		// concat string
		ss << path << "/configuration_file/" << filename;
		// get the final path
		std::string full_path = ss.str();

		std::vector<std::string> path_f;
		path_f.push_back("/usr/local/include/MPCEngine/configuration_file/"+filename);
		path_f.push_back(full_path);
		// Create empty property tree object
		pt::ptree tree;
		for (int i=0; i<path_f.size(); i++)
		{
			try{
				// Parse the XML into the property tree.
				pt::read_xml(path_f[i], tree);
				std::cout << "[Success] Configuration file path found!" << std::endl;
				break;
			}
			catch(std::exception &e) {
				std::cout << "[Warning] Try " << i << ". Configuration file path not found! " << path_f[i] << "\n" << e.what() << std::endl;
			}
		}
		return tree;
    }

    // virtual function
	// initialize the solver if necessary
	virtual Eigen::VectorXd Init(Eigen::VectorXd state_0_in) = 0;
	virtual Eigen::VectorXd ComputeControl(Eigen::VectorXd state_i_in) = 0;
    virtual                 ~MPCproblem(){};

};

typedef std::unique_ptr<MPCproblem> P_unique_MPCinstance;
typedef std::shared_ptr<MPCproblem> P_MPCinstance;





#endif /* MPCPROBLEM_HPP_ */
