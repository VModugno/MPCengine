/*
 * OraclePlanner.hpp
 *
 *  Created on: Dec 20, 2018
 *      Author: vale
 */

#ifndef INCLUDE_API_ORACLEPLANNER_HPP_
#define INCLUDE_API_ORACLEPLANNER_HPP_

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <memory>



class OraclePlan {
public:
    // this method has to return a sequence of state input in this way = (x_0,u_0),(x_1,u_1).....(x_N,U_N)
	// the first is x_0 is the one provided as input
	virtual Eigen::VectorXd ComputePlan(Eigen::VectorXd x0) = 0;
	virtual                 ~OraclePlan(){};
};

typedef std::unique_ptr<OraclePlan> P_unique_oracle;
typedef std::shared_ptr<OraclePlan> P_oracle;



#endif /* INCLUDE_API_ORACLEPLANNER_HPP_ */
