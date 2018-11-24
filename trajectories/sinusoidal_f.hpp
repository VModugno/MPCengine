/*
 * sinusoidal.hpp
 *
 *  Created on: Nov 22, 2018
 *      Author: vale
 */

#ifndef TRAJECTORIES_SINUSOIDAL_F_HPP_
#define TRAJECTORIES_SINUSOIDAL_F_HPP_

#include "all_traj_include.hpp"

inline double sin_f  (double t,int current_step,std::vector<double> param){
	double k = param[0];
    double w = param[1];

    return k*sin(w*t);

};

inline double d_sin_f(double t,int current_step,std::vector<double> param){

	double k = param[0];
	double w = param[1];
	double dt= param[2];

	return k*w*cos(w*t)/dt;
};

inline double cos_f  (double t,int current_step,std::vector<double> param){
	double k = param[0];
    double w = param[1];

    return k*cos(w*t);

};

inline double d_cos_f(double t,int current_step,std::vector<double> param){

	double k = param[0];
	double w = param[1];
	double dt= param[2];

	return -k*w*sin(w*t)/dt;
};



#endif /* TRAJECTORIES_SINUSOIDAL_F_HPP_ */
