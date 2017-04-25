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
KalmanFilter::KalmanFilter() {}

//destructor
KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in, MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Init2(MatrixXd& H_in, MatrixXd& R_in)
{
    H_ = H_in;
    R_ = R_in;
}

void KalmanFilter::Predict()
{
    x_ = (F_ * x_);
    //x_ = (F_ * x_) + v_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd& z)
{
    //compute error
    VectorXd y = z - (H_ * x_);
    //compute s
    MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
    //compute kalman gain
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd& z)
{
    //compute error
    VectorXd y = z - h(x_);

    //std::cout << "y: " << y(1) << std::endl;

    //adjust phi to be between -pi and pi
    if (fabs(y(1)) > PI)
    {
        y(1) -= round(y(1) / (2.0 * PI)) * (2.0 * PI);
    }

    //std::cout << "x_: " << y << std::endl;
    //std::cout << "y: " << y << std::endl;
    //std::cout << "Hj: " << Hj_ << std::endl;
    //std::cout << "Hj: " << Hj_ << std::endl;

    //compute s
    MatrixXd S = (H_ * P_ * H_.transpose()) + R_;

    //std::cout << "Got this far now now..." << std::endl;

    //compute kalman gain
    MatrixXd K = P_ * H_.transpose() * S.inverse();

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

//map x' from cartesian to polar coordinates
VectorXd KalmanFilter::h(const VectorXd& x_state)
{
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    float px_squared = px * px;
    float py_squared = py * py;
    float px_py_sqrt = sqrt(px_squared + py_squared);

    VectorXd h_x(3);
    float phi;

    if (fabs(px) > 0.001)
    {
        phi = atan2(py, px);
    }
    else
    {
        phi = atan2(0.0001, 0.001);
    }

    //compute mapping
    h_x << px_py_sqrt, phi, (((px * vx) + (py * vy)) / px_py_sqrt);

    //std::cout << "h(x): " << h_x(1) << std::endl;

    return h_x;

}
