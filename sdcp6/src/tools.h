/*
######################################################
## AUTHOR: James Beasley                            ##
## DATE: April 10, 2017                             ##
## UDACITY SDC: Project 6 (Extended Kalman Filters) ##
######################################################
*/

#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools
{
	public:
		//constructor
		Tools();
		//destructor
		virtual ~Tools();

		//compute RMSE
		Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

		//compute jacobian
		Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
};

#endif /* TOOLS_H_ */
