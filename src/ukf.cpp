#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define PI 3.14159265

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false; //true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;
  x_.fill(0);
  P_.fill(0);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

	if (!is_initialized_) {
		cout << endl << "Initialized";

		if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			cout << endl << "Initialized LASER";
			x_ <<  meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0,0,0;
			P_ <<
					1000,   0,   0,   0,   0,
					   0,1000,   0,   0,   0,
					   0,   0,1000,   0,   0,
					1000,   0,   0,1000,   0,
					1000,   0,   0,   0,1000;
		}
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			cout << endl << "Initialized RADAR";
			double r = meas_package.raw_measurements_(0);
		    double theta = meas_package.raw_measurements_(1);
		    x_ << r*cos(theta), r*sin(theta), 0,0,0;
			P_ <<
					1000,   0,   0,   0,   0,
					   0,1000,   0,   0,   0,
					   0,   0,1000,   0,   0,
					1000,   0,   0,1000,   0,
					1000,   0,   0,   0,1000;
		}

		is_initialized_ = true;
		time_us_ = meas_package.timestamp_;

		return;
	}

	cout << endl << "Predict";

    double deltaT = (meas_package.timestamp_ - time_us_) / 1000000.0;
    double deltaT2 = deltaT  * deltaT;
    double deltaT3 = deltaT2 * deltaT;
    double deltaT4 = deltaT3 * deltaT;

	cout << endl << "deltaT" << endl << deltaT;

	int n_x = 5;
	int n_aug = 7;
	double lambda = 3 - n_aug;
	VectorXd x_aug = VectorXd(n_aug);
	MatrixXd P_aug = MatrixXd(n_aug,n_aug);
	MatrixXd Xsig_aug = MatrixXd(n_aug, 2*n_aug + 1);

	cout << endl << "x_aug" << endl << x_aug;

	x_aug.fill(0);
	x_aug.block(0,0,n_x,1) = x_;
	P_aug.fill(0);
	P_aug.block(0,0,n_x,n_x) = P_;
	P_aug(5,5) = std_a_ * std_a_;
	P_aug(6,6) = std_yawdd_ * std_yawdd_;
	MatrixXd A = P_aug.llt().matrixL();

	cout << endl << "A" << endl << A;

	double f = sqrt(lambda + n_aug);
	Xsig_aug.block(0,0,n_aug,1) = x_aug;
	for (int i=0; i<7; i++) {
		Xsig_aug.block(0,i + 1 + 0,n_aug,1) = x_aug + f*A.block(0,i,n_aug,1);
		Xsig_aug.block(0,i + 1 + 7,n_aug,1) = x_aug - f*A.block(0,i,n_aug,1);
	}

	cout << endl << "Xsig_aug" << endl << Xsig_aug;

	MatrixXd Xsig_pred = MatrixXd(n_x, 2*n_aug + 1);
	for (int i=0; i<2*n_aug + 1; i++) {
		double px      = Xsig_aug(0,i);
		double py      = Xsig_aug(1,i);
		double v       = Xsig_aug(2,i);
		double phi     = Xsig_aug(3,i);
		double phi_dot = Xsig_aug(4,i);
		double m1      = Xsig_aug(5,i);
		double m2      = Xsig_aug(6,i);

		double px_;
		double py_;
		double v_;
		double phi_;
		double phi_dot_;

		if (fabs(phi) < 0.00001) {
			px_      = px + v * cos(phi) * deltaT + 0.5 * deltaT2 * cos(phi) * m1;
			py_      = py + v * sin(phi) * deltaT + 0.5 * deltaT2 * sin(phi) * m1;
			v_       = v + deltaT * m1;
			phi_     = phi + phi_dot * deltaT + 0.5 * deltaT2 * m2;
			phi_dot_ = phi_dot + deltaT * m2;
		}
		else {
			px_      = px + (v/phi_dot) * ( sin(phi + deltaT*phi_dot) - sin(phi)) + 0.5 * deltaT2 * cos(phi) * m1;
			py_      = py + (v/phi_dot) * (-cos(phi + deltaT*phi_dot) - cos(phi)) + 0.5 * deltaT2 * sin(phi) * m1;
			v_       = v + deltaT * m1;
			phi_     = phi + phi_dot * deltaT + 0.5 * deltaT2 * m2;
			phi_dot_ = phi_dot + deltaT * m2;
		}
		Xsig_pred(0,i) = px_;
		Xsig_pred(1,i) = py_;
		Xsig_pred(2,i) = v_;
		Xsig_pred(3,i) = phi_;
		Xsig_pred(4,i) = phi_dot_;
	}

	cout << endl << "Xsig_pred" << endl << Xsig_pred;

	VectorXd weights = VectorXd(2*n_aug + 1);
	weights(0) = lambda / (lambda + n_aug);
	for (int i=1; i<2*n_aug + 1; i++) {
		weights(i) = 0.5 / (n_aug + lambda);
	}

	cout << endl << "weights" << endl << weights;

	x_.fill(0);
	for (int i=0; i<2*n_aug + 1; i++) {
		x_ = x_ + weights(i)*Xsig_pred.block(0,i,n_x,1);
	}

	cout << endl << "x_" << endl << x_;

	P_.fill(0);
	for (int i=0; i<2*n_aug + 1; i++) {
		P_ = P_ + weights(i)*(Xsig_pred.block(0,i,n_x,1) - x_)*(Xsig_pred.block(0,i,n_x,1) - x_).transpose();
	}

	cout << endl << "P_" << endl << P_;


	if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
		cout << endl << "Process LASER";
	}
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		cout << endl << "Process RADAR";

		int n_z = 3;
		MatrixXd Zsig = MatrixXd(n_z, 2*n_aug+1);
		VectorXd z_pred = VectorXd(n_z);

		for (int i=0; i<2*n_aug + 1; i++) {
			double px      = Xsig_pred(0,i);
			double py      = Xsig_pred(1,i);
			double v       = Xsig_pred(2,i);
			double rho     = Xsig_pred(3,i);
			double rho_dot = Xsig_pred(4,i);

			double r = sqrt(px*px + py*py);
			double alpha = atan2(py,px);
			double r_dot = (px * cos(rho) + py * sin(rho)) / r;

			Zsig(0,i) = r;
			Zsig(1,i) = alpha;
			Zsig(2,i) = r_dot;
		}

		cout << endl << "Zsig" << endl << Zsig;


		z_pred.fill(0);
		for (int i=0; i<2*n_aug + 1; i++) {
			z_pred = z_pred + weights(i)*Zsig.block(0,i,n_z,1);
		}

		cout << endl << "z_pred" << endl << z_pred;


		MatrixXd S = MatrixXd(n_z, n_z);
		S.fill(0);
		for (int i=0; i<2*n_aug + 1; i++) {
			S = S + weights(i)*(Zsig.block(0,i,n_z,1) - z_pred)*(Zsig.block(0,i,n_z,1) - z_pred).transpose();
		}

		cout << endl << "S" << endl << S;

		MatrixXd R = MatrixXd(n_z, n_z);
		R.fill(0);
		R(0,0) = std_radr_ * std_radr_;
		R(1,1) = std_radphi_ * std_radphi_;
		R(2,2) = std_radrd_ * std_radrd_;

		cout << endl << "R" << endl << R;

		S = S + R;

		MatrixXd Tc = MatrixXd(n_z, n_z);
		Tc.fill(0);
		for (int i=0; i<2*n_aug + 1; i++) {
			Tc = Tc + weights(i)*(Xsig_pred.block(0,i,n_x,1) - x_)*(Zsig.block(0,i,n_z,1) - z_pred).transpose();
		}

		cout << endl << "Tc" << endl << Tc;

		MatrixXd K = Tc * S.inverse();

		cout << endl << "K" << endl << K;

		VectorXd z = VectorXd(n_z);
		z <<  meas_package.raw_measurements_(0), meas_package.raw_measurements_(1),  meas_package.raw_measurements_(2);

		cout << endl << "z" << endl << z;

		x_ = x_ + K * (z - z_pred);

		P_ = P_ - K * S * K.transpose();


	}
	time_us_ = meas_package.timestamp_;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
