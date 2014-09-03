#include <kr_math/base_types.hpp>
#include <kr_math/SO3.hpp>
#include <Eigen/Geometry>

#ifndef ERROR_STATE_KF_HPP
#define ERROR_STATE_KF_HPP

/**
 * @class ErrorStateKF
 * @brief Implementation of error-state kalman filter for position and
 * orientation filtering.
 *
 * @note See: "Quaternion kinematics for the error-state KF", Joan Sola
 * @note See: "A Multi-State Constraint Kalman Filter for Vision-aided Inertial
 *Navigation", Mourikis and Roumeliotis
 */
template <typename Scalar> class ErrorStateKF {
public:
  
  /**
   * @brief Construct new instance of ESKF.
   * @note Initializes orientation to identity and all other state variables
   * to zero.
   */
  ErrorStateKF();

  /**
   * @brief Initialize a diagonal covariance matrix.
   * @param qVar Uncertainty on angular orientation (rad).
   * @param bgVar Uncertainty on gyro bias (rad/s).
   * @param vVar Uncertainty on velocity (rad/s).
   * @param baVar Uncertainty on acceleration bias (m/s^2).
   * @param pVar Uncertainty on position (m).
   */
  void initCovariance(Scalar qStd, Scalar bgStd, Scalar vStd, Scalar baStd,
                      Scalar pStd);
  
  /**
   * @brief initState
   * @param wQb
   * @param p
   * @param v
   */
  void initState(const kr::quat<Scalar>& wQb, const kr::vec3<Scalar> p,
                 const kr::vec3<Scalar>& v);

  /**
   * @brief Run the prediction step of the filter.
   * @param wbody Body-frame angular velocity (rad/s).
   * @param varW Covariance of body-frame angular velocity.
   * @param abody Body-frame linear acceleration (m/s^2).
   * @param varA Covariance of body-frame linear acceleration.
   * @param dt Time step (seconds).
   */
  void predict(const kr::vec3<Scalar> &wbody, const kr::mat3<Scalar> &varW,
               const kr::vec3<Scalar> &abody, const kr::mat3<Scalar> &varA,
               Scalar dt);

  /**
   * @brief Run the update step of the filter, with a velocity measurement.
   * @param qm Measured orientation as a quaternion.
   * @param varQ Covariance about x/y/z axes (rad).
   * @param pm Position (meters).
   * @param varP Covariance on position.
   * @param vm Velocity (m/s).
   * @param varV Covariance on velocity.
   * @return True if the update succeeds, false if the kalman gain is singular.
   */
  bool update(const kr::quat<Scalar> &qm, const kr::mat3<Scalar> &varQ,
              const kr::vec3<Scalar> &pm, const kr::mat3<Scalar> &varP,
              const kr::vec3<Scalar> &vm, const kr::mat3<Scalar> &varV);

  /**
   * @brief Orientation, transformation from body to world.
   * @return Quaternion.
   */
  const kr::quat<Scalar>& getOrientation() const { return q_; }
  
  /**
   * @brief Gyro bias estimate, rad/s.
   * @return R3 Vector.
   */
  const kr::vec3<Scalar>& getGyroBias() const { return bg_; }
  
  /**
   * @brief Velocity estimate, m/s.
   * @return R3 Vector.
   */
  const kr::vec3<Scalar>& getVelocity() const { return v_; }
  
  /**
   * @brief Accelerometer bias estimate, m/s^2.
   * @return R3 Vector.
   */
  const kr::vec3<Scalar>& getAccelBias() const { return ba_; }
  
  /**
   * @brief Position estimate, m.
   * @return R3 Vector.
   */
  const kr::vec3<Scalar>& getPosition() const { return p_; }
  
  /**
   * @brief Get system covariance.
   * @return 15x15 matrix.
   * @note Order of elements: [theta, gyro bias, vel, accel bias, pos]
   */
  const kr::mat<Scalar,15,15>& getCovariance() const { return P_; }
  
  /**
   * @brief Set std dev. of bias drift rates.
   * @param bg Gyro bias drift rate uncertainty, rad/s^2.
   * @param ba Accelerometer bias drift rate uncertainty, m/s^3.
   */  
  void setBiasUncertainties(Scalar bg, Scalar ba) {
    Qbg_.setIdentity();
    Qbg_ *= bg;
    Qba_.setIdentity();
    Qba_ *= ba;
  }
  
  /**
   * @brief Set the magnitude of the gravity vector.
   * @param g Gravity magnitude, m/s^2.
   */
  void setGravity(Scalar g) {
    g_ = g;
  }
  
private:
  kr::quat<Scalar> q_;  /// Orientation
  kr::vec3<Scalar> bg_; /// Gyro bias
  kr::vec3<Scalar> v_;  /// Velocity
  kr::vec3<Scalar> ba_; /// Accelerometer bias
  kr::vec3<Scalar> p_;  /// Position

  kr::mat<Scalar, 15, 15> P_; /// State covariance

  kr::mat3<Scalar> Qbg_; /// Gyro bias drift rate uncertainty
  kr::mat3<Scalar> Qba_; /// Accel bias drift rate uncertainty

  Scalar g_; /// Gravity constant
};

template <typename Scalar>
ErrorStateKF<Scalar>::ErrorStateKF() : g_(9.80665) {
  q_.setIdentity();
  bg_.setZero();
  v_.setZero();
  ba_.setZero();
  p_.setZero();

  P_.setZero();
  Qbg_.setZero();
  Qba_.setZero();
}

template <typename Scalar>
void ErrorStateKF<Scalar>::initCovariance(Scalar qStd, Scalar bgStd,
                                          Scalar vStd, Scalar baStd,
                                          Scalar pStd) {
  static const kr::mat3<Scalar> I3 = kr::mat3<Scalar>::Identity();
  
  P_.template block<3,3>(0,0) = I3*qStd*qStd;
  P_.template block<3,3>(3,3) = I3*bgStd*bgStd;
  P_.template block<3,3>(6,6) = I3*vStd*vStd;
  P_.template block<3,3>(9,9) = I3*baStd*baStd;
  P_.template block<3,3>(12,12) = I3*pStd*pStd;
}

template <typename Scalar>
void ErrorStateKF<Scalar>::initState(const kr::quat<Scalar>& wQb, 
                                     const kr::vec3<Scalar> p,
                                     const kr::vec3<Scalar>& v) {
  q_ = wQb;
  p_ = p;
  v_ = v;
}

template <typename Scalar>
void ErrorStateKF<Scalar>::predict(const kr::vec3<Scalar> &wbody,
                                   const kr::mat3<Scalar> &varW,
                                   const kr::vec3<Scalar> &abody,
                                   const kr::mat3<Scalar> &varA, Scalar dt) {

  const kr::vec3<Scalar> ah = abody - ba_; //  corrected inputs
  const kr::vec3<Scalar> wh = wbody - bg_;
  const kr::mat3<Scalar> wRb = q_.matrix(); //  current orientation

  //  integrate state forward w/ euler equations
  const kr::quat<Scalar> dq =
      q_ * kr::quat<Scalar>(0, wh[0] * 0.5, wh[1] * 0.5, wh[2] * 0.5);
  q_.w() += dq.w() * dt;
  q_.x() += dq.x() * dt;
  q_.y() += dq.y() * dt;
  q_.z() += dq.z() * dt;
  q_.normalize();
  p_ += v_ * dt;
  v_ += (wRb * ah + kr::vec3<Scalar>(0, 0, -g_)) * dt;

  //  construct error-state jacobian
  kr::mat<Scalar, 15, 15> F;
  F.setZero();

  ///  dth = [wh] x dth - bg
  F.template block<3, 3>(0, 0) = -kr::skewSymmetric(wh);                  
  F.template block<3, 3>(0, 3) = -kr::mat3<Scalar>::Identity();

  //  dv = -R[ah] x dth - R[ba]
  F.template block<3, 3>(6, 0) = -wRb * kr::skewSymmetric(ah);            
  F.template block<3, 3>(6, 9) = -wRb;

  //  dp = dv
  F.template block<3, 3>(12, 6).setIdentity(); 

  //  form process covariance matrix
  kr::mat<Scalar, 12, 12> Q;
  Q.setZero();
  Q.template block<3, 3>(0, 0) = varW;
  Q.template block<3, 3>(3, 3) = Qbg_;
  Q.template block<3, 3>(6, 6) = varA;
  Q.template block<3, 3>(9, 9) = Qba_;
  
  kr::mat<Scalar, 15, 12> G;
  G.setZero();

  //  angular vel. variance on error state angle
  G.template block<3, 3>(0, 0) = kr::mat<Scalar, 3, 3>::Identity() * -1;
  G.template block<3, 3>(3, 3).setIdentity();
  G.template block<3, 3>(6, 6) = -wRb; //  acceleration on linear velocity
  G.template block<3, 3>(9, 9).setIdentity();

  //  integrate covariance forward
  P_ += (F*P_ + P_*F.transpose() + G*Q*G.transpose()) * dt;
}

template <typename Scalar>
bool ErrorStateKF<Scalar>::update(const kr::quat<Scalar> &qm,
                                  const kr::mat3<Scalar> &varQ,
                                  const kr::vec3<Scalar> &pm,
                                  const kr::mat3<Scalar> &varP,
                                  const kr::vec3<Scalar> &vm,
                                  const kr::mat3<Scalar> &varV) {
  //  form the measurement jacobian
  kr::mat<Scalar, 6, 15> H;
  H.setZero();

  //  position and velocity
  H.template block<3, 3>(0, 12).setIdentity();
  H.template block<3, 3>(3, 6).setIdentity();

  //  orientation
  //H.template block<3, 3>(0, 0).setIdentity();

  // residual
  kr::mat<Scalar, 6, 1> r;

  //  non-linear rotation residual
  const kr::quat<Scalar> dq = q_.conjugate() * qm;
  const Eigen::AngleAxis<Scalar> aa(dq);
  //r.template block<3, 1>(0, 0) =
  //    aa.angle() * aa.axis();

  //  linear pos/velocity
  r.template block<3, 1>(0, 0) = pm - p_;
  r.template block<3, 1>(3, 0) = vm - v_;

  //  measurement covariance
  kr::mat<Scalar, 6, 6> R;
  R.setZero();
  //R.template block<3, 3>(0, 0) = varQ;
  R.template block<3, 3>(0, 0) = varP;
  R.template block<3, 3>(3, 3) = varV;

  //  kalman update
  kr::mat<Scalar, 6, 6> S = H * P_ * H.transpose() + R;
  auto LU = S.fullPivLu();
  if (!LU.isInvertible()) {
    return false;
  }
  S = LU.inverse();

  const kr::mat<Scalar, 15, 6> K = P_ * H.transpose() * S;
  const kr::mat<Scalar, 15, 1> dx = K * r;

  P_ = (kr::mat<Scalar, 15, 15>::Identity() - K * H) * P_;

  //  update state
  q_ *= kr::quat<Scalar>(1, dx[0] * 0.5, dx[1] * 0.5, dx[2] * 0.5);
  q_.normalize();
  bg_ += dx.template block<3, 1>(3, 0);
  v_ += dx.template block<3, 1>(6, 0);
  ba_ += dx.template block<3, 1>(9, 0);
  p_ += dx.template block<3, 1>(12, 0);

  return true;
}

#endif // ERROR_STATE_KF_HPP
