/*
 * recorded_f.hpp
 *
 *  Created on: Nov 24, 2018
 *      Author: vale
 */

#ifndef TRAJECTORIES_RECORDED_F_HPP_
#define TRAJECTORIES_RECORDED_F_HPP_

#include "all_traj_include.hpp"

// in this case param contains the entire set of values we want to reproduce
//param[0]   = ref_1
//   .
//   .
//   .
//param[n_i] = ref_n_i



inline double recorded_f(double t,int current_step,std::vector<double> param){

    return param[current_step];

};


#endif /* TRAJECTORIES_RECORDED_F_HPP_ */
