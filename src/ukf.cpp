#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Normalize angles in radians to [-PI, PI].
 */
static double normalize_angle(const double radians)
{
    // Copy the sign of the value in radians to the value of pi.
    const double signed_pi = std::copysign(M_PI, radians);
    // Set the value of difference to the appropriate signed value between pi and -pi.
    return std::fmod(radians + signed_pi, M_PI * 2) - signed_pi;
}



/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Integration timestep
  timestep_ = 0.05;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // We are tracking a bicycle, let's put a maximum accel of 1 m/s^2. Then,
  // the rule of thumb says that the noise would be 0.5 m/s^2.
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // Again, let's try to guess a reasonable value for a bicycle.
  std_yawdd_ = 0.8;

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

  // State dimension
  n_x_ = 5;

  // We select the sigma points using the Van der Merwe's algorithm.
  // See Rudolph Van der Merwe. "Sigma-Point Kalman Filters for
  //  Probabilistic Inference in Dynamic State-Space Models" (2004).

  // initial state vector [pos1 pos2 vel_abs yaw_angle yaw_rate]
  x_ = VectorXd::Zero(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Predicted sigma points, initialized to zero
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  // Spreading parameter
  lambda_ = 3 - n_x_;
  // This is equivalent to alpha = 1, beta = 2 and kappa = 3 - n_x_
  // in Van der Merwe's algorithm for scaled sigma points:
  //    `lambda_ = alpha * alpha * (n_x_ + kappa) - n_x_`

  // Weights of sigma points

  // weight for the mean
  weights_M_ = VectorXd(2 * n_aug_ + 1);
  weights_M_(0) = lambda_ / (lambda_ + n_aug_);
  weights_M_.tail(2 * n_aug_).fill(0.5 / (lambda_ + n_aug_));

  weights_C_ = VectorXd(2 * n_aug_ + 1);
  // weights for the covariance
  weights_C_(0) = lambda_ / (lambda_ + n_aug_) + /* beta */ 2;
  weights_C_.tail(2 * n_aug_).fill(0.5 / (lambda_ + n_aug_));

  // Measurement matrix for laser
  H_laser_ = MatrixXd::Zero(2, n_x_);
  H_laser_(0, 0) = 1;
  H_laser_(1, 1) = 1;

  // Measurment covariance matrix for laser
  R_laser_ = MatrixXd::Zero(2, 2);
  R_laser_(0, 0) = std_laspx_ * std_laspx_;
  R_laser_(1, 1) = std_laspy_ * std_laspy_;

  // Measurment covariance matrix for radar
  R_radar_ = MatrixXd::Zero(3, 3);
  R_radar_(0, 0) = std_radr_ * std_radr_;
  R_radar_(1, 1) = std_radphi_ * std_radphi_;
  R_radar_(2, 2) = std_radrd_ * std_radrd_;

  I_ = MatrixXd::Identity(n_x_, n_x_);

  // TODO
  time_us_ = 0;
  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;

  // Waiting for first measurement
  is_initialized_ = false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 * @return true if measurement was used.
 */
bool UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (!is_initialized_) {
    // Initialize state means and covariance
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar ...
      const double rho = meas_package.raw_measurements_(0);
      const double phi = meas_package.raw_measurements_(1);
      // ... to cartesian coordinates
      const double px = rho * cos(phi);
      const double py = rho * sin(phi);

      // Note that the radar velocity and the CTRV velocity are not the same.
      // CRTV is tangencial velocity while RADAR velocity is radial.
      //const double rho_dot = meas_package.raw_measurements_(2);

      // Set the state: {pos1 pos2 vel_abs yaw_angle yaw_rate}
      x_ << px, py, 0, 0, 0;
    }
    else { // LIDAR
      // Already cartesian coordinates
      const double px = meas_package.raw_measurements_(0);
      const double py = meas_package.raw_measurements_(1);
      // Set the state: {pos1 pos2 vel_abs yaw_angle yaw_rate}
      x_ << px, py, 0, 0, 0;
    }
    // Set initial timestamp
    time_us_ = meas_package.timestamp_;
    // Initialization completed
    is_initialized_ = true;
    return true;
  }

  // Time elapsed between measurements, in seconds
  double delta_t = (meas_package.timestamp_ - time_us_) * 1e-6;
  while (delta_t > timestep_) {
    Prediction(timestep_);
    delta_t -= timestep_;
  }
  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    if (!use_radar_)
      return false;

    // No need to update for invalid measurements
    const double rho = meas_package.raw_measurements_(0);
    if (fabs(rho) < 1e-3)
      return false;

    UpdateRadar(meas_package);
  } else { // LASER
    if (!use_laser_)
      return false;

    // No need to update for invalid measurements
    const double px = meas_package.raw_measurements_(0);
    const double py = meas_package.raw_measurements_(1);
    if ((px*px + py*py) < 1e-6)
      return false;

    UpdateLidar(meas_package);
  }

  // Update state time
  time_us_ = meas_package.timestamp_;
  return true;
}

#ifdef USE_NUMERICAL_INTEGRATION
static VectorXd SystemDynamics(VectorXd x, VectorXd nu)
{
  const double
    px       = x(0),
    py       = x(1),
    v        = x(2),
    yaw      = x(3),
    yawd     = x(4);
  const double
    nu_a     = nu(0),
    nu_yawdd = nu(1);

  (void)px;
  (void)py;

  VectorXd dx = VectorXd(5);
  dx << v * cos(yaw),
        v * sin(yaw),
        nu_a,
        yawd,
        nu_yawdd;
  return dx;
}

static VectorXd IntegrateEuler(VectorXd state, VectorXd nu, double dt)
{
  auto initial = SystemDynamics(state, nu);
  return state + dt * initial;
}

static VectorXd IntegrateHeun(VectorXd state, VectorXd nu, double dt)
{
  auto initial = SystemDynamics(state, nu);
  auto predictor =  state + dt * initial;
  return state + (dt * 0.5) * (initial + SystemDynamics(predictor, nu));
}

static VectorXd IntegrateRK4(VectorXd state, VectorXd nu, double dt)
{
  auto a = SystemDynamics(state, nu);
  auto b = SystemDynamics(state + 0.5 * dt * a, nu);
  auto c = SystemDynamics(state + 0.5 * dt * b, nu);
  auto d = SystemDynamics(state + dt * c, nu);
  return state + (dt / 6) * (a + 2 * b + 2 * c + d);
}
#endif


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  // 1. Generate sigma points - encode posterior uncertainty

  // Augmented mean state
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(n_x_) = x_;

  // Augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  // State covariance
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  // Process covariance matrix
  P_aug(n_x_    , n_x_    ) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // Square root of P_aug
  const MatrixXd A_aug = P_aug.llt().matrixL();

  // Augmented sigma points, {x, x +/- sqrt(lambda + n) * A}
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  // First is the mean of the input
  Xsig_aug.colwise() = x_aug;
  Xsig_aug.block(0, 1       , n_aug_, n_aug_) += sqrt(lambda_ + n_aug_) * A_aug;
  Xsig_aug.block(0, 1+n_aug_, n_aug_, n_aug_) -= sqrt(lambda_ + n_aug_) * A_aug;

  // 2. Predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
#ifdef USE_NUMERICAL_INTEGRATION
    Xsig_pred_.col(i) = IntegrateRK4(Xsig_aug.col(i).head(5),
                                     Xsig_aug.col(i).tail(2), dt);
#else
    // Extract values for better readability
    const double p_x      = Xsig_aug(0, i);
    const double p_y      = Xsig_aug(1, i);
    const double v        = Xsig_aug(2, i);
    const double yaw      = Xsig_aug(3, i);
    const double yawd     = Xsig_aug(4, i);
    const double nu_a     = Xsig_aug(5, i);
    const double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p, v_p, yaw_p, yawd_p;

    // avoid division by zero
    if (fabs(yawd) > 1e-3) {
      px_p = p_x + v / yawd * ( sin(yaw + yawd * dt) - sin(yaw));
      py_p = p_y + v / yawd * (-cos(yaw + yawd * dt) + cos(yaw));
    }
    else {
      px_p = p_x + v * dt * cos(yaw);
      py_p = p_y + v * dt * sin(yaw);
    }

    v_p = v;
    yaw_p = yaw + yawd * dt;
    yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * dt*dt * cos(yaw);
    py_p = py_p + 0.5 * nu_a * dt*dt * sin(yaw);
    v_p = v_p + nu_a * dt;

    yaw_p = yaw_p + 0.5 * nu_yawdd * dt*dt;
    yawd_p = yawd_p + nu_yawdd * dt;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
#endif
  }

  // 3. Predict mean and covariance - recover prior uncertainty

  // State mean prediction
  x_ = Xsig_pred_ * weights_M_;
  // State difference
  MatrixXd X_diff = Xsig_pred_.colwise() - x_;
  // Normalize yaw_angle to [-PI, PI]
  X_diff.row(3) = X_diff.row(3).unaryExpr(&normalize_angle);
  // State covariance matrix prediction
  P_ = X_diff * weights_C_.asDiagonal() * X_diff.transpose();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // The LIDAR measurement is linear, hence we just can use the standard
  // linear Kalman equations for the update. The UKF equations will give
  // the same result but LKF is more efficient.

  // Measurement matrix
  const MatrixXd H_ = H_laser_;
  // Measurement covariances
  const MatrixXd R_ = R_laser_;
  // Kalman gain
  const MatrixXd S_inv = (H_ * P_ * H_.transpose() + R_).inverse();
  const MatrixXd K = P_ * H_.transpose() * S_inv;
  // Measurement prediction
  const VectorXd z_pred = H_ * x_;
  // Residual
  const VectorXd z_res = meas_package.raw_measurements_ - z_pred;
  // New prior
  x_ = x_ + K * z_res;
  P_ = (I_ - K * H_) * P_;

 // Normalized Innovation Squared ~ \Xi^2
  NIS_laser_ = z_res.transpose() * S_inv * z_res;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // The RADAR measurement is non-linear, we use the UKF equations.

  // 1. Predict the measurement
  const int n_z = 3;

  const auto px  = Xsig_pred_.row(0).array();
  const auto py  = Xsig_pred_.row(1).array();
  const auto v   = Xsig_pred_.row(2).array();
  const auto psi = Xsig_pred_.row(3).array();

  MatrixXd Zsig(n_z, 2 * n_aug_ + 1);

  // rho.
  Zsig.row(0) = (px.square() + py.square()).sqrt()
                  // avoid division by zero when px and py are too small
                  .unaryExpr([](double v){ return fmax(v, 1e-4); });
  // phi = atan2(py, px). Saddly, Eigen doesn't have an atan2 function
  Zsig.row(1) = py.binaryExpr(px, static_cast<double(*)(double, double)>(&std::atan2));
  // rho_dot.
  Zsig.row(2) = ((px * cos(psi) + py * sin(psi)) * v) / Zsig.row(0).array();

  // Calculate mean predicted measurement z = [rho, phi, rho_dot]
  VectorXd z_pred = (Zsig * weights_M_);

  //
  MatrixXd z_diff = Zsig.colwise() - z_pred;
  // Normalize angles
  z_diff.row(1) = z_diff.row(1).unaryExpr(&normalize_angle);

  // Calculate measurement covariance matrix S
  MatrixXd S = z_diff * weights_C_.asDiagonal() * z_diff.transpose()
             + R_radar_ /* noise */
             ;

  const MatrixXd S_inv = S.inverse();

  // 2. Update the state with the new measurement

  // Compute the cross correlation matrix
  MatrixXd x_diff = Xsig_pred_.colwise() - x_;
  // Normalize yaw_angle to [-PI, PI]
  x_diff.row(3) = x_diff.row(3).unaryExpr(&normalize_angle);
  MatrixXd Tc = x_diff * weights_C_.asDiagonal() * z_diff.transpose();

  //  Kalman gain
  const MatrixXd K = Tc * S_inv;

  const VectorXd z_res = meas_package.raw_measurements_ - z_pred;

  // New prior
  x_ = x_ + K * z_res;
  P_ = P_ - K * S * K.transpose();

  // Normalized Innovation Squared ~ \Xi^2
  NIS_radar_ = z_res.transpose() * S_inv * z_res;
}
