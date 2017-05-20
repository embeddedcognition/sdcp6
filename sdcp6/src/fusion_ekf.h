/*
######################################################
## AUTHOR: James Beasley                            ##
## DATE: April 10, 2017                             ##
## UDACITY SDC: Project 6 (Extended Kalman Filters) ##
######################################################
*/

#ifndef FUSION_EKF_H_
#define FUSION_EKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF
{
    public:
        //constructor
        FusionEKF();
        //destructor
        virtual ~FusionEKF();
        //execute the kalman filter on the current sensor measurement
        void ProcessMeasurement(const MeasurementPackage& measurement_pack);
        //return the current state vector x --> (px, py, vx, vy)
        Eigen::VectorXd GetState();

    private:
        //function that handles first time init
        void FirstTimeInit(const MeasurementPackage& measurement_pack);
        //object containing kalman filter logic
        KalmanFilter kf_;
        //check whether the tracking toolbox was initiallized or not (first measurement)
        bool is_initialized_;
        //previous timestamp
        long long previous_timestamp_;
        //tool object used to compute Jacobian and RMSE
        Tools tools_;
        //R and H matrices differ based on sensor type
        Eigen::MatrixXd R_laser_;
        Eigen::MatrixXd R_radar_;
        Eigen::MatrixXd H_laser_;
        Eigen::MatrixXd Hj_;
};

#endif /* FUSION_EKF_H_ */
