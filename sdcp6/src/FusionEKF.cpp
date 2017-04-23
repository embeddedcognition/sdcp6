/*
######################################################
## AUTHOR: James Beasley                            ##
## DATE: April 10, 2017                             ##
## UDACITY SDC: Project 6 (Extended Kalman Filters) ##
######################################################
*/

#include <iostream>
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//constructor
FusionEKF::FusionEKF()
{
	//bool to invoke initialization on first "ProcessMeasurement" call
	is_initialized_ = false;
	//previous measurement timestamp
	previous_timestamp_ = 0;

	// initializing matrices
	ekf_.x_ = VectorXd(4);		//state vector (px, py, vx, vy)
	ekf_.F_ = MatrixXd(4, 4);	//state transition matrix
	ekf_.v_ = VectorXd(4);		//state transition noise vector (vpx, vpy, vvx, vvy)
	ekf_.Q_ = MatrixXd(4, 4);	//process covariance (noise)
	R_laser_ = MatrixXd(2, 2);  //measurement covariance (noise) for laser
	R_radar_ = MatrixXd(3, 3);  //measurement covariance (noise) for radar
	H_laser_ = MatrixXd(2, 4);  //belief projection matrix for lidar
	Hj_ = MatrixXd(3, 4);		//jacobian for radar

	//set measurement covariance (noise) matrix for laser
	R_laser_ << 0.0225, 0,
              	0, 0.0225;
	//set measurement covariance (noise) matrix for radar
	R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
	//set the state transition matrix (only two locations (those equal to 5) continuously change, so we'll change them per iteration vs. reinitializing the whole matrix each time)
	ekf_.F_ << 1, 0, 5, 0,
	           0, 1, 0, 5,
	           0, 0, 1, 0,
			   0, 0, 0, 1;
	//set acceleration noise components
	noise_ax = 9;
	noise_ay = 9;
}

//destructor
FusionEKF::~FusionEKF() {}

//process the latest measurement received from the sensor
void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
	//if this is the first time, we need to initialize x with the current measurement, then exit (filtering will start with the next iteration)
	if (!is_initialized_)
	{
		//for laser: col_1 will equal px and col_2 will equal py
		//for radar: col_1 will equal r (e.g., rho) and col_2 will equal θ (e.g., phi)
		float col_1 = measurement_pack.raw_measurements_(0);
		float col_2 = measurement_pack.raw_measurements_(1);
		float px;
		float py;

		//if this is radar data
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		{
			//we need to convert from polar coordinates (r, θ) to cartesian coordinates (x, y), via x = r * cos(θ) and y = r * sin(θ)
			px = col_1 * cos(col_2);
			py = col_1 * sin(col_2);
		}
		//if this is lidar data
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		{
			//we're already in cartesian coordinates, so just assign to px and py
			px = col_1;
			py = col_2;
		}

		//init state vector x, which is in cartesian coordinates (px, py, vx, vy)
		//since we'll not have enough information to initialize the velocity portion of the state vector (vx and vy), i.e., we only have the current position to go on, we'll set it to zero
		ekf_.x_ << px, py, 0, 0;

		//capture the timestamp of the measurement for future use
		previous_timestamp_ = measurement_pack.timestamp_;

		//done initializing
		is_initialized_ = true;
		//no need to predict or update so we return
		return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // Radar updates
  }
  else
  {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
