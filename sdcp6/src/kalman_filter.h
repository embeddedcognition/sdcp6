/*
######################################################
## AUTHOR: James Beasley                            ##
## DATE: April 10, 2017                             ##
## UDACITY SDC: Project 6 (Extended Kalman Filters) ##
######################################################
*/

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter
{
    public:
        //constructor
        KalmanFilter();
        //destructor
        virtual ~KalmanFilter();

        //set the appropriate H & R matrices based upon the sensor type
        void SetMeasurementMatrices(Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in);

        //update the state transition matrix F based on the new elapsed time (delta t)
        void UpdateStateTransitionMatrix(const float& dt);

        //update the process covariance matrix Q based on the new elapsed time (delta t)
        void UpdateProcessCovarianceMatrix(const float& dt);

        //update the state vector x --> (px, py, vx, vy)
        void InitState(const float px, const float py, const float vx, const float vy);

        //return the current state vector x --> (px, py, vx, vy)
        Eigen::VectorXd GetState();

        //return the current state covariance matrix
        Eigen::MatrixXd GetStateCovariance();

       /*
        * Predicts the state and the state covariance
        * using the process model
        * @param delta_T Time between k and k+1 in s
        */
        void PerformPredict();

       /*
        * Updates the state by using standard Kalman Filter equations
        * @param z The measurement at k+1
        */
        void PerformUpdate(const Eigen::VectorXd& z);

       /*
        * Updates the state by using Extended Kalman Filter equations
        * @param z The measurement at k+1
        */
        void PerformUpdateEKF(const Eigen::VectorXd& z);

    private:
        //helper function for completing the measurement update
        void PerformUpdateHelper(const Eigen::VectorXd& y);

        //h(x) function --> map x' from cartesian coordinates to polar coordinates (extended kalman filter only)
        Eigen::VectorXd h(const Eigen::VectorXd& x);

        //normalize the supplied angle to be within -pi to pi
        double NormalizeAngle(const double angle);

        //state vector
        Eigen::VectorXd x_;
        //state covariance matrix
        Eigen::MatrixXd P_;
        //state transistion matrix
        Eigen::MatrixXd F_;
        //process covariance matrix
        Eigen::MatrixXd Q_;
        //measurement matrix
        Eigen::MatrixXd H_;
        //measurement covariance matrix
        Eigen::MatrixXd R_;

        //const for PI
        const double PI = 3.14159265358979;

        //acceleration noise components
        float noise_ax;
        float noise_ay;
};

#endif /* KALMAN_FILTER_H_ */
