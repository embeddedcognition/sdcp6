/*
######################################################
## AUTHOR: James Beasley                            ##
## DATE: April 10, 2017                             ##
## UDACITY SDC: Project 6 (Extended Kalman Filters) ##
######################################################
*/

#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage
{
    public:
        long long timestamp_;

        enum SensorType
        {
            LASER,
            RADAR
        } sensor_type_;

        Eigen::VectorXd gt_values_;
};

#endif /* GROUND_TRUTH_PACKAGE_H_ */
