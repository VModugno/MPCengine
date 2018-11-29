#ifndef ABSTSTRACT_ENV
#define ABSTSTRACT_ENV

#include <Eigen/Core>
#include <Eigen/LU>    // necessary for matrix inversion in Eigen
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

struct DynComp{

	Eigen::MatrixXd M;
	Eigen::MatrixXd S;
	Eigen::VectorXd C;
	Eigen::VectorXd g;
	DynComp(){};
	DynComp(int DOF):M(DOF,DOF),S(DOF,DOF),C(DOF),g(DOF){
		//this->M = Eigen::MatrixXd::Zero(DOF,DOF);
		//this->S = Eigen::MatrixXd::Zero(DOF,DOF);
		//this->C = Eigen::VectorXd::Zero(DOF);
		//this->g = Eigen::VectorXd::Zero(DOF);
	};
};

typedef std::shared_ptr<DynComp> P_DynComp;


class abstractEnv {
public:

	int                          dim_state;
	int                          DOF;
	Eigen::MatrixXd              state_bounds;
	std::vector<std::string>     state_name;
	Eigen::VectorXd              init_state;
	Eigen::VectorXd              state;
	double                       dt;                         // sample duration (s)
	double                       ft;                         // final time      (s)
	bool                         active_visualization;
	Eigen::VectorXd              mes_acc;
	bool                         feedback_lin;               // whit this field i specify if the system require a feedback linearization or not
	// logging vector
	bool                         log = false;
	std::vector<Eigen::VectorXd> states;
	std::vector<Eigen::VectorXd> actions;
	P_DynComp                    comps;
	bool                         already_discretized=false;
    // integrating dynamics with runge-kutta 4
	double Step(Eigen::VectorXd action, Eigen::VectorXd & new_state, Eigen::VectorXd & mes_acc)
	{
		double reward = 0;


		if(!already_discretized){
			Eigen::VectorXd k1(dim_state),k2(dim_state),k3(dim_state),k4(dim_state);
			// to fix
			int substeps = 1;
			//runge-kutta 4
			for (int i=0;i<substeps;i++)
			{
				k1 = this->Dynamics(this->state,action,this->mes_acc);
				//DEBUG
				//std::cout << "k1  = " << k1 << std::endl;
				k2 = this->Dynamics(this->state+this->dt/2*k1,action);
				//DEBUG
				//std::cout << "k2  = " << k2 << std::endl;
				k3 = this->Dynamics(this->state+this->dt/2*k2,action);
				//DEBUG
				//std::cout << "k3  = " << k3 << std::endl;
				k4 = this->Dynamics(this->state+this->dt*k3,action);
				//DEBUG
				//std::cout << "k4  = " << k4 << std::endl;

				new_state = this->state + this->dt/6*(k1 + 2*k2 + 2*k3 + k4);
				//All states wrapped to 2pi (when necessary)
				this->Wrapping(new_state);
				//DEBUG
				//std::cout << "new_state = " << new_state << std::endl;
			}
		}
		else{
			new_state = this->Dynamics(this->state,action);
			this->Wrapping(new_state);
		}

		this->state = new_state; // Old state = new state
		// TODO reintroduce reward functions with a class of reward functions
		//reward    = this->reward(new_state,action);
		//done      = 1;

		if(this->active_visualization)
			this->UpdateRender(new_state);

		// here i log the data collected while performing the experiments
		if(this->log){
			this->states.push_back(new_state);
			this->actions.push_back(action);
		}

		return reward;
	}

	void logToFile() {

        std::string filename_state  = "state_from_mpc_cpp.mat";
        std::string filename_action = "action_from_mpc_cpp.mat";
		// i create a string stream for concatenating strings
		std::stringstream ss_state,ss_action;
		// i get the current working directory
		fs::path pathfs = fs::current_path();
		// convert to a string
		std::string path = pathfs.string();
		// concat string
		ss_state  << path <<  "/mpc_preprocessing/@log/" << filename_state;
		ss_action << path <<  "/mpc_preprocessing/@log/" << filename_action;

		// get the final path
		std::string full_path_state  = ss_state.str();
		std::string full_path_action = ss_action.str();

		std::ofstream file_state(full_path_state);
		std::ofstream file_action(full_path_action);
		// with this eigen class i define the format for plottin eigen vector
		Eigen::IOFormat fm(Eigen::FullPrecision);

	  if (file_state.is_open())
	  {
		for(unsigned int i = 0; i < this->states.size();i++){
			file_state << states[i].transpose().format(fm) << std::endl;
		}

	  }
	  if (file_action.is_open())
	  {
		  for(unsigned int i = 0; i < this->actions.size();i++){
		  			file_action << actions[i].transpose().format(fm) << std::endl;
		  		}
	  }


	}

	void DysplayComp(){
		std::cout <<"M = "<< this->comps->M << std::endl;
		std::cout <<"S = "<< this->comps->S << std::endl;
		std::cout <<"C = "<< this->comps->C << std::endl;
		std::cout <<"g = "<< this->comps->g << std::endl;
	};

    // virtual function
	// with this function we collect the acceleration to simulate measure of acceleration
	virtual P_DynComp       GetDynamicalComponents(Eigen::VectorXd cur_state) = 0;
	virtual Eigen::VectorXd Dynamics(Eigen::VectorXd state,Eigen::VectorXd action, Eigen::VectorXd & mes_acc) = 0;
	// with this function i do not update the mes_action
	virtual Eigen::VectorXd Dynamics(Eigen::VectorXd state,Eigen::VectorXd action) = 0;
	virtual void            Wrapping(Eigen::VectorXd & state) = 0;
	virtual void            plotInfoEnv() = 0;
	virtual void            Load_parameters(Eigen::VectorXd params) = 0;
	virtual void            Render() = 0;
    virtual void            UpdateRender(Eigen::VectorXd state) = 0;
    virtual                 ~abstractEnv(){};

};

typedef std::unique_ptr<abstractEnv> P_unique_env;

#endif /* ABSTSTRACT_ENV  */
