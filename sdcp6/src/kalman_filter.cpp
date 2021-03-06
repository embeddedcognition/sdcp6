/*
######################################################
## AUTHOR: James Beasley                            ##
## DATE: April 10, 2017                             ##
## UDACITY SDC: Project 6 (Extended Kalman Filters) ##
######################################################
*/

#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

//constructor
KalmanFilter::KalmanFilter()
{
    // initializing matrices
    x_ = VectorXd(4);      //state vector (px, py, vx, vy)
    F_ = MatrixXd(4, 4);   //state transition matrix
    P_ = MatrixXd(4, 4);   //state covariance matrix
    Q_ = MatrixXd(4, 4);   //process covariance (noise)

    //set the state transition matrix (only two locations (those equal to 5) continuously change,
    //so we'll change them per iteration vs. reinitializing the whole matrix each time)
    F_ << 1, 0, 5, 0,
          0, 1, 0, 5,
          0, 0, 1, 0,
          0, 0, 0, 1;
    //set the state covariance matrix
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;
    //set the process covariance matrix (only eight locations (those equal to 5) continuously change,
    //so we'll change them per iteration vs. reinitializing the whole matrix each time)
    Q_ << 5, 0, 5, 0,
          0, 5, 0, 5,
          5, 0, 5, 0,
          0, 5, 0, 5;

    //set acceleration noise components
    noise_ax = 9;
    noise_ay = 9;
}

//destructor
KalmanFilter::~KalmanFilter() {}

//set the appropriate H & R matrices based upon the sensor type
void KalmanFilter::SetMeasurementMatrices(MatrixXd& H_in, MatrixXd& R_in)
{
    H_ = H_in;
    R_ = R_in;
}

//update the transition state matrix F
void KalmanFilter::UpdateStateTransitionMatrix(const float& dt)
{
    //update the state transition matrix according to the new elapsed time
    //only the following matrix locations need to change
    F_(0, 2) = dt;
    F_(1, 3) = dt;
}

//update the process covariance matrix Q
void KalmanFilter::UpdateProcessCovarianceMatrix(const float& dt)
{
    //local vars
    float dt_squared;
    float dt_cubed;
    float dt_to_the_fourth;

    //compute multi-use operations
    dt_squared = dt * dt;
    dt_cubed = dt_squared * dt;
    dt_to_the_fourth = dt_cubed * dt;

    //update the process covariance matrix according to the new elapsed time
    //only the following matrix locations need to change
    Q_(0, 0) = ((dt_to_the_fourth / 4) * noise_ax);
    Q_(0, 2) = ((dt_cubed / 2) * noise_ax);
    Q_(1, 1) = ((dt_to_the_fourth / 4) * noise_ay);
    Q_(1, 3) = ((dt_cubed / 2) * noise_ay);
    Q_(2, 0) = ((dt_cubed / 2) * noise_ax);
    Q_(2, 2) = (dt_squared * noise_ax);
    Q_(3, 1) = ((dt_cubed / 2) * noise_ay);
    Q_(3, 3) = (dt_squared * noise_ay);
}

//init the state vector x --> (px, py, vx, vy), just on first measurement
void KalmanFilter::InitState(const float px, const float py, const float vx, const float vy)
{
    x_ << px, py, vx, vy;
}

//return the current state vector x --> (px, py, vx, vy)
VectorXd KalmanFilter::GetState()
{
    return x_;
}

//return the current state covariance matrix
MatrixXd KalmanFilter::GetStateCovariance()
{
    return P_;
}

//perform kalman prediction step
void KalmanFilter::PerformPredict()
{
    //predict state
    x_ = (F_ * x_);
    //predict covariance
    P_ = F_ * P_ * F_.transpose() + Q_;
}

//perform kalman update step using standard equations (lidar only)
void KalmanFilter::PerformUpdate(const VectorXd& z)
{
    //local vars
    VectorXd y;

    //compute error --> find the difference between the latest sensor measurement (z) and our prediction x'
    //that has been mapped by the measurement matrix H (e.g., velocity is dropped and we're only comparing position),
    //as lidar only measures position
    y = z - (H_ * x_);

    //finish completing update (shared code between standard and EKF)
    PerformUpdateHelper(y);
}

//perform kalman update step using extended equations (radar only)
void KalmanFilter::PerformUpdateEKF(const VectorXd& z)
{
    //local vars
    VectorXd y;

    //compute error --> find the difference between the latest sensor measurement (z)
    //and our prediction x' that has been mapped from cartesian to polar coordinates by function h
    y = z - h(x_);

    //adjust phi to be between -pi and pi (if necessary)
    y(1) = NormalizeAngle(y(1));

    //finish completing update (shared code between standard and EKF)
    PerformUpdateHelper(y);
}

//helper function for completing the measurement update (instead of duplicating code in both update functions)
void KalmanFilter::PerformUpdateHelper(const Eigen::VectorXd& y)
{
    //local vars
    MatrixXd S;
    MatrixXd K;
    long x_size;
    MatrixXd I;

    //compute sensitivity
    S = (H_ * P_ * H_.transpose()) + R_;
    //compute kalman gain
    K = P_ * H_.transpose() * S.inverse();

    //update estimate and uncertainty
    x_ = x_ + (K * y);
    x_size = x_.size();
    I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

//h(x) function --> map x' from cartesian coordinates to polar coordinates (extended kalman filter only)
VectorXd KalmanFilter::h(const VectorXd& x_state)
{
    //local vars
    float px;
    float py;
    float vx;
    float vy;
    float px_squared;
    float py_squared;
    float sqrt_px_squared_plus_py_squared;
    float phi;
    VectorXd h_x(3);  //vector containing mapped polar values for comparison against current radar measurement

    //recover state parameters
    px = x_state(0);
    py = x_state(1);
    vx = x_state(2);
    vy = x_state(3);

    //compute multi-use operations
    px_squared = px * px;
    py_squared = py * py;
    sqrt_px_squared_plus_py_squared = sqrt(px_squared + py_squared);

    //check for divide by zero for rho dot
    if (sqrt_px_squared_plus_py_squared < 0.0001)
    {
        sqrt_px_squared_plus_py_squared = 0.0001;
    }

    //check for divide by zero for phi
    if (fabs(px) < 0.0001)
    {
        phi = atan2(py, 0.0001);
    }
    else
    {
        phi = atan2(py, px);
    }

    //compute mapping
    h_x << sqrt_px_squared_plus_py_squared, phi, (((px * vx) + (py * vy)) / sqrt_px_squared_plus_py_squared);

    //return polar matrix
    return h_x;
}

//normalize the supplied angle to be within -pi to pi
double KalmanFilter::NormalizeAngle(const double angle)
{
    //local vars
    double normalized_angle = angle;

    //adjust phi to be between -pi and pi
    //http://stackoverflow.com/questions/11980292/how-to-wrap-around-a-range
    if (fabs(angle) > PI)
    {
        double two_pi = 2 * PI;
        normalized_angle -= round(normalized_angle / two_pi) * two_pi;
    }

    return normalized_angle;
}
