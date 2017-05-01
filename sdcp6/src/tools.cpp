/*
######################################################
## AUTHOR: James Beasley                            ##
## DATE: April 10, 2017                             ##
## UDACITY SDC: Project 6 (Extended Kalman Filters) ##
######################################################
*/

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

//constructor
Tools::Tools() {}

//destructor
Tools::~Tools() {}

//compute the rmse of our estimations vs. ground truth
VectorXd Tools::ComputeRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth)
{
    //local vars
    VectorXd rmse(4);

    //init rmse (returned if we have invalid input)
    rmse << 0, 0, 0, 0;

    //check the validity of the following inputs:
    //* the estimation vector size should not be zero
    //* the estimation vector size should equal ground truth vector size
    if ((estimations.size() != ground_truth.size()) || (estimations.size() == 0))
    {
        cout << "Invalid estimation or ground_truth data." << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i)
    {
        //compute residual
        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        //add residual
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the resulting rmse
    return rmse;
}

//compute jacobian for use in extended kalman filter
MatrixXd Tools::ComputeJacobian(const VectorXd& x_state)
{
    //local vars
    MatrixXd Hj(3, 4);
    float px;
    float py;
    float vx;
    float vy;
    float px_squared;
    float py_squared;
    float px_squared_plus_py_squared;

    //recover state parameters
    px = x_state(0);
    py = x_state(1);
    vx = x_state(2);
    vy = x_state(3);

    //compute multi-use operations
    px_squared = px * px;
    py_squared = py * py;
    px_squared_plus_py_squared = px_squared + py_squared;

    //check division by zero
    if (px_squared_plus_py_squared > 0.0001)
    {
        //compute the Jacobian matrix
        Hj << (px / sqrt(px_squared_plus_py_squared)), (py / sqrt(px_squared_plus_py_squared)), 0, 0,
              -(py / (px_squared_plus_py_squared)), (px / (px_squared_plus_py_squared)), 0, 0,
              ((py * ((vx * py) - (vy * px))) / sqrt(pow(px_squared_plus_py_squared, 3))), ((px * ((vy * px) - (vx * py))) / sqrt(pow(px_squared_plus_py_squared, 3))), (px / sqrt(px_squared_plus_py_squared)), (py / sqrt(px_squared_plus_py_squared));
    }
    else
    {
        cout << "Avoiding divide by zero, Jacobian not calculated...zeroed out matrix returned." << endl;

        //init Hj (if we catch ourselves dividing by zero, return this version of the matrix)
        Hj << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;
    }

    //return matrix
    return Hj;
}
