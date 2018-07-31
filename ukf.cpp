#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;
    
    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true; //true;
    
    // initial state vector
    x_ = VectorXd(5);
    
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    
    
    // PRIMEIRA SOLUCAO   sta_a_ = 7.0  e std_yawdd_ = 0.9
    // SEGUNDA SOLUCAO   sta_a_ = 0.9  e std_yawdd_ = 0.5
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.9;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.5;
    
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
            x_ <<  meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0,0,0;
        }
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double r = meas_package.raw_measurements_(0);
            double theta = meas_package.raw_measurements_(1);
            x_ << r*cos(theta), r*sin(theta), 0,0,0;
        }
        
        P_ <<
        10,   0,   0,   0,   0,
        0,10,   0,   0,   0,
        0,   0, 10,   0,   0,
        0,   0,   0, 10,   0,
        0,   0,   0,   0, .01;
        
        weights_ = VectorXd(2*n_aug + 1);
        weights_(0) = 1.0 / (2*n_aug + 1);
        weights_(0) = lambda / (lambda + n_aug);
        for (int i=1; i<2*n_aug + 1; i++) {
            weights_(i) = 1.0 / (2*n_aug + 1);
            weights_(i) = 0.5 / (n_aug + lambda);
        }
        
        
        is_initialized_ = true;
        time_us_ = meas_package.timestamp_;
        
        return;
    }
    
    double deltaT = (meas_package.timestamp_ - time_us_) / 1000000.0;
    
    Prediction(deltaT);
    
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(meas_package);
    }
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(meas_package);
    }
    time_us_ = meas_package.timestamp_;
    
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double deltaT) {
    /**
     TODO:
     
     Complete this function! Estimate the object's location. Modify the state
     vector, x_. Predict sigma points, the state, and the state covariance matrix.
     */
    
    
    VectorXd x_aug = VectorXd(n_aug);
    MatrixXd P_aug = MatrixXd(n_aug,n_aug);
    MatrixXd Xsig_aug = MatrixXd(n_aug, 2*n_aug + 1);
    
    x_aug.fill(0);
    x_aug.block(0,0,n_x,1) = x_;
    x_aug(3) = tools.AdjustAngle(x_aug(3));
    
    P_aug.fill(0);
    P_aug.block(0,0,n_x,n_x) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;
    
    
    MatrixXd A = P_aug.llt().matrixL();
    
    double f = sqrt(lambda + n_aug);
    Xsig_aug.block(0,0,n_aug,1) = x_aug;
    for (int i=0; i<n_aug; i++) {
        Xsig_aug.block(0,i + 1 + 0,    n_aug,1) = x_aug + f*A.block(0,i,n_aug,1);
        Xsig_aug.block(0,i + 1 + n_aug,n_aug,1) = x_aug - f*A.block(0,i,n_aug,1);
    }
    
    Xsig_pred_ = MatrixXd(n_x, 2*n_aug + 1);
    for (int i=0; i<2*n_aug + 1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*deltaT) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*deltaT) );
        }
        else {
            px_p = p_x + v*deltaT*cos(yaw);
            py_p = p_y + v*deltaT*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*deltaT;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5*nu_a*deltaT*deltaT * cos(yaw);
        py_p = py_p + 0.5*nu_a*deltaT*deltaT * sin(yaw);
        v_p = v_p + nu_a*deltaT;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*deltaT*deltaT;
        yawd_p = yawd_p + nu_yawdd*deltaT;
        
        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
    
    x_.fill(0);
    for (int i=0; i<2*n_aug + 1; i++) {
        x_ = x_ + weights_(i)*Xsig_pred_.block(0,i,n_x,1);
    }
    x_(3) = tools.AdjustAngle(x_(3));
    
    P_.fill(0);
    for (int i=0; i<2*n_aug + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = tools.AdjustAngle(x_diff(3));
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
    
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
    
    int n_z = 2;
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug+1);
    VectorXd z_pred = VectorXd(n_z);
    
    for (int i=0; i<2*n_aug + 1; i++) {
        double px      = Xsig_pred_(0,i);
        double py      = Xsig_pred_(1,i);
        Zsig(0,i) = px;
        Zsig(1,i) = py;
    }
    
    z_pred.fill(0);
    for (int i=0; i<2*n_aug + 1; i++) {
        z_pred = z_pred + weights_(i)*Zsig.block(0,i,n_z,1);
    }
    
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    MatrixXd R = MatrixXd(n_z, n_z);
    R.fill(0);
    R(0,0) = std_laspx_ * std_laspx_;
    R(1,1) = std_laspy_ * std_laspy_;
    
    S = S + R;
    
    MatrixXd Tc = MatrixXd(n_x, n_z);
    Tc.fill(0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        VectorXd z_diff = Zsig.col(i) - z_pred;
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    MatrixXd K = Tc * S.inverse();
    
    VectorXd z = VectorXd(n_z);
    z <<  meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);
    
    VectorXd z_diff = z - z_pred;
    
    x_ = x_ + K * z_diff;
    
    P_ = P_ - K * S * K.transpose();

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
    
    int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug+1);
    VectorXd z_pred = VectorXd(n_z);
    
    for (int i=0; i<2*n_aug + 1; i++) {
        double px      = Xsig_pred_(0,i);
        double py      = Xsig_pred_(1,i);
        double v       = Xsig_pred_(2,i);
        double rho     = Xsig_pred_(3,i);
        
        double r = sqrt(px*px + py*py);
        double alpha = atan2(py,px);
        double r_dot = v * (px * cos(rho) + py * sin(rho)) / r;
        
        Zsig(0,i) = r;
        Zsig(1,i) = alpha;
        Zsig(2,i) = r_dot;
    }
    
    z_pred.fill(0);
    for (int i=0; i<2*n_aug + 1; i++) {
        z_pred = z_pred + weights_(i)*Zsig.block(0,i,n_z,1);
    }
    z_pred(1) = tools.AdjustAngle(z_pred(1));
    
    
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = tools.AdjustAngle(z_diff(1));
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    MatrixXd R = MatrixXd(n_z, n_z);
    R.fill(0);
    R(0,0) = std_radr_ * std_radr_;
    R(1,1) = std_radphi_ * std_radphi_;
    R(2,2) = std_radrd_ * std_radrd_;
    
    S = S + R;
    
    MatrixXd Tc = MatrixXd(n_x, n_z);
    Tc.fill(0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
        VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = tools.AdjustAngle(z_diff(1));
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = tools.AdjustAngle(x_diff(3));
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    MatrixXd K = Tc * S.inverse();
    
    
    VectorXd z = VectorXd(n_z);
    z <<  meas_package.raw_measurements_(0), meas_package.raw_measurements_(1),  meas_package.raw_measurements_(2);
    
    VectorXd z_diff = z - z_pred;
    z_diff(1) = tools.AdjustAngle(z_diff(1));
    
    x_ = x_ + K * z_diff;
    x_(3) = tools.AdjustAngle(x_(3));
    
    P_ = P_ - K * S * K.transpose();

}
