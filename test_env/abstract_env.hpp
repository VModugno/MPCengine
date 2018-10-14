#include <Eigen/Core>


class abstractEnv {
public:

	int num_state;
	Eigen::MatrixXd state_bounds;
	std::vector<std::string> state_name;
	Eigen::VectorXd init_state;
	Eigen::VectorXd state;
	double dt;
	bool active_visualization;

    // integrating dynamics with runge-kutta 4
	double Step(Eigen::VectorXd action, Eigen::VectorXd & mes_acc, Eigen::VectorXd & new_state)
	{
		double reward;
		int    done;
		Eigen::VectorXd k1,k2,k3,k4;

		// to fix
		int substeps = 2;
		//runge-kutta 4
		for (int i=0;i<substeps;i++)
		{
			k1 = this->Dynamics(this->state,action);
			k2 = this->Dynamics(this->state+this->dt/2*k1,action);
			k3 = this->Dynamics(this->state+this->dt/2*k2,action);
			k4 = this->Dynamics(this->state+this->dt*k3,action);

			new_state = this->state + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);
			//All states wrapped to 2pi
			new_state = this->Wrapping(new_state);
		}

		this->state = new_state; // Old state = new state
		new_state = this->state;
		reward    = this->reward(new_state,action);
		done      = 1;

		if(this->active_visualization)
			this->UpdateRender(new_state);
	}

    // virtual function
	virtual Eigen::VectorXd Dynamics(Eigen::VectorXd action, Eigen::VectorXd & mes_acc) = 0;
	virtual Eigen::VectorXd Wrapping(Eigen::VectorXd state) = 0;
	virtual void            Load_parameters(Eigen::VectorXd params) = 0;
	virtual void            Render() = 0;
    virtual void            UpdateRender(Eigen::VectorXd state) = 0;

};
