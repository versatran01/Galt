#ifndef QUADROTOR_UKF_H
#define QUADROTOR_UKF_H

#include <ros/ros.h>
#include <Eigen/Core>

class QuadrotorUKF {
 public:
  // Dimensions
  static const int state_count_ = 12;
  static const int process_noise_count_ = 9;
  static const int meas_gps_count_ = 7;
  static const int input_count_ = 6;

  typedef Eigen::Matrix<double, state_count_, 1> StateVec;
  typedef Eigen::Matrix<double, state_count_, state_count_> StateCov;
  typedef Eigen::Matrix<double, process_noise_count_, 1> ProcNoiseVec;
  typedef Eigen::Matrix<double, process_noise_count_, process_noise_count_>
      ProcNoiseCov;
  typedef Eigen::Matrix<double, meas_gps_count_, 1> MeasGPSVec;
  typedef Eigen::Matrix<double, meas_gps_count_, meas_gps_count_> MeasGPSCov;
  typedef Eigen::Matrix<double, input_count_, 1> InputVec;

  QuadrotorUKF();
  ~QuadrotorUKF();

  const StateVec& GetState();
  const StateCov& GetStateCovariance();
  ros::Time GetStateTime();

  void SetGravity(double _g);
  void SetImuCovariance(const ProcNoiseCov& _Rv);
  void SetParameters(double _alpha, double _beta, double _kappa);

  bool ProcessUpdate(const InputVec& u, ros::Time time);
  bool MeasurementUpdateGPS(const MeasGPSVec& z, const MeasGPSCov& RnGPS,
                            ros::Time time);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Dimensions
  static const int L_ = state_count_ + process_noise_count_;

  // Private functions
  void GenerateWeights();
  void GenerateSigmaPoints();
  const StateVec ProcessModel(const StateVec& x, const InputVec& u,
                              const ProcNoiseVec& v, double dt);
  const MeasGPSVec MeasurementModelGPS(const StateVec& x);
  bool PropagateAprioriCovariance(const ros::Time time);

  // State
  StateVec xa_;
  StateCov Pa_;
  Eigen::Matrix<double, state_count_, 2 * L_ + 1> Xa_;
  Eigen::Matrix<double, process_noise_count_, 2 * L_ + 1> Va_;
  ros::Time xa_time_;

  // Initial process update indicator
  bool init_process_;
  bool init_meas_;

  // Measurement update indicator
  bool meas_update_flag_;

  // Previous state
  StateVec pxa_;
  StateCov pPa_;
  ros::Time pxa_time_;

  // Process Covariance Matrix
  ProcNoiseCov Rv_;

  // Gravity
  double g_;

  // UKF Parameters
  double alpha_;
  double beta_;
  double kappa_;
  double lambda_;
  double gamma_;

  // UKF Weights
  Eigen::Matrix<double, 1, 2 * L_ + 1> wm_;
  Eigen::Matrix<double, 1, 2 * L_ + 1> wc_;
};

#endif
