#include <Eigen/Core>
#include <vector>
#include <string>

class abstractEnv {
public:

	int num_state;
	Eigen::MatrixXd state_bounds;
	std::vector<std::string> state_name;
	Eigen::VectorXd init_state;
	Eigen::VectorXd state;
	double dt;
	bool active_visualization;
	Eigen::VectorXd mes_acc;

    // integrating dynamics with runge-kutta 4
	double Step(Eigen::VectorXd action, Eigen::VectorXd & new_state, Eigen::VectorXd & mes_acc)
	{
		double reward = 0;
		Eigen::VectorXd k1(num_state),k2(num_state),k3(num_state),k4(num_state);

		// to fix
		int substeps = 1;
		//runge-kutta 4
		for (int i=0;i<substeps;i++)
		{
			k1 = this->Dynamics(this->state,action,this->mes_acc);
			k2 = this->Dynamics(this->state+this->dt/2*k1,action);
			k3 = this->Dynamics(this->state+this->dt/2*k2,action);
			k4 = this->Dynamics(this->state+this->dt*k3,action);

			new_state = this->state + this->dt/6*(k1 + 2*k2 + 2*k3 + k4);
			//All states wrapped to 2pi (when necessary)
			this->Wrapping(new_state);
		}

		this->state = new_state; // Old state = new state
		// TODO reintroduce reward functions with a class of reward functions
		//reward    = this->reward(new_state,action);
		//done      = 1;

		if(this->active_visualization)
			this->UpdateRender(new_state);

		return reward;
	}

    // virtual function
	// with this function we collect the acceleration to simulate measure of acceleration
	virtual Eigen::VectorXd Dynamics(Eigen::VectorXd state,Eigen::VectorXd action, Eigen::VectorXd & mes_acc) = 0;
	// with this function i do not update the mes_action
	virtual Eigen::VectorXd Dynamics(Eigen::VectorXd state,Eigen::VectorXd action) = 0;
	virtual void            Wrapping(Eigen::VectorXd & state) = 0;
	virtual void            Load_parameters(Eigen::VectorXd params) = 0;
	virtual void            Render() = 0;
    virtual void            UpdateRender(Eigen::VectorXd state) = 0;
    virtual                 ~abstractEnv(){};

};
