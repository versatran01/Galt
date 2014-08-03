#include "quadrotor_ukf.h"
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

QuadrotorUKF::QuadrotorUKF()
{
  // Init State
  xa_.setZero();
  Xa_.setZero();
  Va_.setZero();
  Pa_.setZero();
  Pa_(0,0)   = 0.5*0.5;
  Pa_(1,1)   = 0.5*0.5;
  Pa_(2,2)   = 0.1*0.1;
  Pa_(3,3)   = 0.1*0.1;
  Pa_(4,4)   = 0.1*0.1;
  Pa_(5,5)   = 0.1*0.1;
  Pa_(6,6)   = 10*M_PI/180*10*M_PI/180;
  Pa_(7,7)   = 10*M_PI/180*10*M_PI/180;
  Pa_(8,8)   = 10*M_PI/180*10*M_PI/180;
  Pa_(9,9)   = 0.01*0.01;
  Pa_(10,10) = 0.01*0.01;
  Pa_(11,11) = 0.01*0.01;
  Rv_.setIdentity();
  // Init Sigma Points
  alpha_ = 0.1;
  beta_  = 2;
  kappa_ = 0;
  GenerateWeights();
  // Other Inits
  g_ = QuadrotorUKF::kOneG;
  init_process_ = false;
  init_meas_ = false;
}

const QuadrotorUKF::StateVec &QuadrotorUKF::GetState()
{
  return xa_;
}

ros::Time QuadrotorUKF::GetStateTime()
{
  return xa_time_;
}

const QuadrotorUKF::StateCov &QuadrotorUKF::GetStateCovariance()
{
  return Pa_;
}

void QuadrotorUKF::SetGravity(double _g)
{
  g_ = _g;
}

void QuadrotorUKF::SetImuCovariance(const ProcNoiseCov& _Rv)
{
  Rv_ = _Rv;
}

void QuadrotorUKF::SetParameters(double _alpha, double _beta, double _kappa)
{
  alpha_ = _alpha;
  beta_  = _beta;
  kappa_ = _kappa;
  GenerateWeights();
}

bool QuadrotorUKF::ProcessUpdate(const InputVec &u, ros::Time time)
{
  // Init Time
  double dt;
  if(!init_process_ || !init_meas_)
  {
    init_process_ = true;
    xa_time_ = time;
    return false;
  }

  dt = (time-xa_time_).toSec();
  xa_time_ = time;

  // Generate sigma points
  GenerateSigmaPoints();
  // Mean
  for(unsigned int k = 0; k < 2*L_+1; k++)
    Xa_.col(k) = ProcessModel(Xa_.col(k), u, Va_.col(k), dt);
  // Handle jump between +pi and -pi !
  double minYaw = Xa_.row(6).minCoeff(); //as_scalar(min(Xa_.row(6), 1));
  double maxYaw = Xa_.row(6).maxCoeff(); //as_scalar(max(Xa_.row(6), 1));
  if(std::abs(minYaw - maxYaw) > M_PI)
  {
    for(unsigned int k = 0; k < 2*L_+1; k++)
      if (Xa_(6,k) < 0)
        Xa_(6,k) += 2*M_PI;
  }
  // Now we can get the mean...
  xa_ = wm_.replicate<state_count_, 1>().cwiseProduct(Xa_).rowwise().sum();
  // Covariance
  Pa_.setZero();
  for(unsigned int k = 0; k < 2*L_+1; k++)
  {
    StateVec d = Xa_.col(k) - xa_;
    Pa_.noalias() += wc_(k) * d * d.transpose();
  }
#if 0
  // Just update state, defer covariance update
  ProcNoiseVec v = ProcNoiseVec::Zero();
  xa_ = ProcessModel(xa_, u, v, dt);
#endif
  return true;
}

bool QuadrotorUKF::MeasurementUpdateGPS(const MeasGPSVec &z, const MeasGPSCov &RnGPS, ros::Time time)
{
  // FIXME: Init
  if(!init_process_ || !init_meas_)
  {
    xa_.topRows<meas_gps_count_>()  = z.topRows<meas_gps_count_>();
    init_meas_ = true;
    return false;
  }

  // Get Measurement
  MeasGPSVec za = MeasurementModelGPS(xa_);
  Eigen::Matrix<double, meas_gps_count_, state_count_> H;
  H.setZero();
  H(0,0) = 1;
  H(1,1) = 1;
  H(2,2) = 1;
  H(3,3) = 1;
  H(4,4) = 1;
  H(5,5) = 1;
  H(6,6) = 1;
  MeasGPSCov S = H * Pa_ * H.transpose() + RnGPS;
  // Kalman Gain;
  Eigen::Matrix<double, state_count_, meas_gps_count_> K = Pa_ * H.transpose() * S.inverse();
  // Innovation
  MeasGPSVec inno = z - za;
  // Handle angle jumps
  inno(6) = std::asin(std::sin(inno(6)));
  // Posterior Mean
  xa_ += K * inno;
  // Posterior Covariance
  Pa_ = Pa_ - K * H * Pa_;

  return true;
}

void QuadrotorUKF::GenerateWeights()
{
  lambda_ = alpha_*alpha_*(L_+kappa_)-L_;
  wm_(0) = lambda_ / (L_+lambda_);
  wc_(0) = lambda_ / (L_+lambda_) + (1-alpha_*alpha_+beta_);
  for(unsigned int k = 1; k <= 2*L_; k++)
  {
    wm_(k) = 1 / (2 * (L_+lambda_));
    wc_(k) = 1 / (2 * (L_+lambda_));
  }
  gamma_ = std::sqrt(L_ + lambda_);
}

void QuadrotorUKF::GenerateSigmaPoints()
{
  // Expand state
  Eigen::Matrix<double, L_, 1> xaa = Eigen::Matrix<double, L_, 1>::Zero();
  xaa.topRows<state_count_>() = xa_;
  Eigen::Matrix<double, L_, L_> Paa = Eigen::Matrix<double, L_, L_>::Zero();
  Paa.block<state_count_, state_count_>(0, 0) = Pa_;
  Paa.block<process_noise_count_, process_noise_count_>(state_count_, state_count_) = Rv_;

  // Matrix square root
  Eigen::Matrix<double, L_, L_> sqrtPaa = Paa.llt().matrixL(); //trans(chol(Paa));

  Eigen::Matrix<double, L_, 2*L_+1> Xaa = xaa.replicate<1,2*L_+1>();
  Xaa.block<L_, L_>(0, 1).noalias()   += gamma_ * sqrtPaa;
  Xaa.block<L_, L_>(0, L_+1).noalias() -= gamma_ * sqrtPaa;

  // Push back to original state
  Xa_ = Xaa.topRows<state_count_>();
  Va_ = Xaa.bottomRows<process_noise_count_>();
}

static const Eigen::Matrix3d ypr_to_R(const Eigen::Vector3d &ypr)
{
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(ypr(0), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(ypr(1), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(ypr(2), Eigen::Vector3d::UnitX());
  return R;
}

static const Eigen::Vector3d R_to_ypr(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d ypr;
  ypr(0) = atan2(R(1,0), R(0,0));
  ypr(1) = -asin(R(2,0));
  ypr(2) = atan2(R(2,1), R(2,2));
  return ypr;
}

const QuadrotorUKF::StateVec QuadrotorUKF::ProcessModel(const StateVec& x, const InputVec& u, const ProcNoiseVec& v,
                                                        double dt)
{
  Eigen::Matrix3d R = ypr_to_R(x.segment<3>(6));
  Eigen::Vector3d ag;
  ag(0) = 0;
  ag(1) = 0;
  ag(2) = g_;
  // Acceleration
  Eigen::Vector3d a = u.segment<3>(0) + v.segment<3>(0);
  Eigen::Vector3d ddx = R * (a - x.segment<3>(9)) - ag;
  // Rotation
  Eigen::Vector3d w = u.segment<3>(3) + v.segment<3>(3);
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  dR(0,1) = -w(2) * dt;
  dR(0,2) =  w(1) * dt;
  dR(1,0) =  w(2) * dt;
  dR(1,2) = -w(0) * dt;
  dR(2,0) = -w(1) * dt;
  dR(2,1) =  w(0) * dt;
  Eigen::Matrix3d Rt = R * dR;
  // State
  StateVec xt = x;
  xt.segment<3>(0) = x.segment<3>(0) + x.segment<3>(3)*dt + ddx*dt*dt/2;
  xt.segment<3>(3) =                   x.segment<3>(3)    + ddx*dt     ;
  xt.segment<3>(6) = R_to_ypr(Rt);
  xt.segment<3>(9) = x.segment<3>(9) + v.segment<3>(6)*dt;
  return xt;
}

const QuadrotorUKF::MeasGPSVec QuadrotorUKF::MeasurementModelGPS(const StateVec& x)
{
  MeasGPSVec z = x.topRows<meas_gps_count_>();
  return z;
}

#if 0
bool QuadrotorUKF::PropagateAprioriCovariance(const ros::Time time)
{
  if(!init_process_ || !init_meas_)
    return false;

  // Time, rot, gravity
  double dt = (xa_time_ - pxa_time_).toSec();

  if(std::abs(dt) < 0.001)
    return false;

  Eigen::Matrix3d pR = ypr_to_R(pxa_.segment<3>(6));
  Eigen::Vector3d ag;
  ag(0) = 0;
  ag(1) = 0;
  ag(2) = g_;
  // Linear Acceleration
  Eigen::Vector3d dv = xa_.segment<3>(3) - pxa_.segment<3>(3);
  Eigen::Vector3d a = pR.transpose() * (dv / dt + ag) + pxa_.segment<3>(9);
  // Angular Velocity
  Eigen::Matrix3d dR = pR.transpose() * ypr_to_R(xa_.segment<3>(6));
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  w(0) = dR(2,1) / dt;
  w(1) = dR(0,2) / dt;
  w(2) = dR(1,0) / dt;
  // Assemble state and control
  InputVec u;
  u << a, w;
  StateVec _xa = xa_;
  xa_ = pxa_;
  // Generate sigma points
  GenerateSigmaPoints();
  // Mean
  for(unsigned int k = 0; k < 2*L_+1; k++)
    Xa_.col(k) = ProcessModel(Xa_.col(k), u, Va_.col(k), dt);
  // Handle jump between +pi and -pi !
  double minYaw = Xa_.row(6).minCoeff(); //as_scalar(min(Xa_.row(6), 1));
  double maxYaw = Xa_.row(6).maxCoeff(); //as_scalar(max(Xa_.row(6), 1));
  if(std::abs(minYaw - maxYaw) > M_PI)
  {
    for(unsigned int k = 0; k < 2*L_+1; k++)
      if (Xa_(6,k) < 0)
        Xa_(6,k) += 2*M_PI;
  }
  // Now we can get the mean...
  xa_ = wm_.replicate<state_count_, 1>().cwiseProduct(Xa_).rowwise().sum();
  // Covariance
  Pa_.setZero();
  for(unsigned int k = 0; k < 2*L_+1; k++)
  {
    StateVec d = Xa_.col(k) - xa_;
    Pa_.noalias() += wc_(k) * d * d.transpose();
  }
  xa_ = _xa;

  return true;
}
#endif
