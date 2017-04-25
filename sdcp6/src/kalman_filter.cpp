/*
######################################################
## AUTHOR: James Beasley                            ##
## DATE: April 10, 2017                             ##
## UDACITY SDC: Project 6 (Extended Kalman Filters) ##
######################################################
*/

#include "kalman_filter.h"

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

void KalmanFilter::Predict()
{
    x_ = (F_ * x_) + v_;
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
    Tools tools;
    MatrixXd Hj(3, 4);

    //get the jacobian
    Hj = tools.CalculateJacobian(x_);

    //compute error
    VectorXd y = z - h(x_);
    //compute s
    MatrixXd S = (Hj * P_ * Hj.transpose()) + R_;
    //compute kalman gain
    MatrixXd K = P_ * Hj.transpose() * S.inverse();

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj) * P_;
}

//map x' from cartesian to polar coordinates
VectorXd KalmanFilter::h(const VectorXd& x_state)
{

}
