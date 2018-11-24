/*
 * trajectories.hpp
 *
 *  Created on: Nov 22, 2018
 *      Author: vale
 */

#ifndef TRAJECTORIES_HPP_
#define TRAJECTORIES_HPP_

#include <Eigen/Core>
#include <Eigen/LU>    // necessary for matrix inversion in Eigen
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <functional>
#include "trajectories/all_traj.hpp"

typedef std::function< double(double, int, std::vector<double> )> traj;
typedef std::vector<std::vector<double> >  param_vec;

class trajectories {

	bool online_comp;                              // in this we select to compute everything offline or to recompute ref each time
	double dt;                                     // time step
	double ft;                                     // final time
	int pred_window;                               // width of prediction window (in number of sample)
	param_vec parameters;  // trajectories parameters
	//void (*traj)(int);
	std::vector<traj> refs;                        // in the refs structure the order of the function matters!!!
	Eigen::VectorXd numerical_ref;                 // it contains the numerical value of the current references


public:
	trajectories(){};
	trajectories(double dt,double ft,int pred_w,std::vector<traj> refs,param_vec param,bool online_comp){
		this->dt          = dt;
		this->ft          = ft;
		this->pred_window = pred_w;
		this->refs        = refs;
		this->parameters  = param;
		this->online_comp = online_comp;
		if(online_comp){
			this->numerical_ref = Eigen::VectorXd::Zero(refs.size()*this->pred_window);
			// DEBUG
			std::cout << "numerical_ref.size() = " << numerical_ref.size() << std::endl;
			std::cout << "numerical_ref = " << numerical_ref << std::endl;
		}
		else{
			// here i have to precompute the vector numerical_ref
		}
	}


	Eigen::VectorXd ComputeTraj(double curr_time,int curr_sample){


		if(online_comp){

			for(int i=0;i<this->pred_window;i=i+refs.size()){
				//DEBUG
				std::cout << "i = "<<i<<std::endl;
				for(unsigned int j=0;j<refs.size();j++){
					//DEBUG
					std::cout << "j = "<<j<<std::endl;
					double res        = refs[j](curr_time,curr_sample,parameters[j]);
					//DEBUG
					std::cout << "double res  = "<<res<<std::endl;
					this->numerical_ref(i+j)= res;

				}
				curr_time   = curr_time + this->dt;
				curr_sample = curr_sample + 1;
			}


			return numerical_ref;
		}else{
		// here i have to split the numerical_ref eigen::vec accordingly to current step the prediction window and the number of output to control
		}


	}

	// GET function
	bool GetOnlineComp(){return this->online_comp;}
	double GetDt      (){return this->dt;};


};



#endif /* TRAJECTORIES_HPP_ */
