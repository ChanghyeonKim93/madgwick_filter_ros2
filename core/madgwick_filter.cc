#include "madgwick_filter.h"

#include <iostream>

namespace madgwick_filter {

MadgwickFilter::MadgwickFilter(const Parameters& parameters)
    : parameters_(parameters) {}

MadgwickFilter::~MadgwickFilter() {}

void MadgwickFilter::Update(const bridge::Imu& bridge_imu) {
  constexpr double kGravitationalAcceleration{9.81};
  constexpr double kBeta{0.4};
  constexpr double kGravAccDiffThreshold{1e-1};

  const auto imu = ConvertFromBridge(bridge_imu);

  if (!condition_.is_bias_initialized) {
    constexpr int kMinNumImuData{50};
    imu_queue_.push_back(imu);
    if (imu_queue_.size() > kMinNumImuData)
      condition_.is_bias_initialized = InitializeGyroBias();
    return;
  }

  // Propagation via angular velocity
  if (prev_time_ == 0.0) {
    prev_time_ = imu.time;
    return;
  }

  const double dt = imu.time - prev_time_;
  const Quaternion dq_w =
      ComputeGradientByAngularVelocity(imu.angular_velocity - gyro_bias_);

  // Compensate orientation using linear acceleration only when the norm
  // of acceleration is near the gravity.
  Eigen::Vector3d am = imu.linear_acceleration - acc_bias_;
  const bool update_by_acceleration =
      std::abs(am.norm() - kGravitationalAcceleration) < kGravAccDiffThreshold;

  Quaternion dq = dq_w;
  if (update_by_acceleration) {
    std::cerr << "Update By Acc.\n";
    const Quaternion dq_a = ComputeGradientlByLinearAcceleration(am);
    dq = dq_w + dq_a * (-kBeta);
  }
  orientation_ += dq * dt;
  orientation_.normalize();

  prev_time_ = imu.time;
}

bridge::Orientation MadgwickFilter::GetOrientation() const {
  const auto& q = orientation_;
  bridge::Orientation bridge_orientation{q.w, q.x, q.y, q.z};
  return bridge_orientation;
}

bool MadgwickFilter::InitializeGyroBias() {
  // TODO(@): estimate acc bias
  gyro_bias_.setZero();
  for (const auto& imu : imu_queue_) gyro_bias_ += imu.angular_velocity;
  gyro_bias_ /= static_cast<double>(imu_queue_.size());
  return true;
}

Quaternion MadgwickFilter::ComputeGradientByAngularVelocity(
    const Vec3& angular_velocity) {
  const auto wm = angular_velocity - gyro_bias_;
  const Quaternion o_w(0.0, wm.x(), wm.y(), wm.z());
  const Quaternion dq_w = (orientation_ * o_w) * 0.5;
  return dq_w;
}

Quaternion MadgwickFilter::ComputeGradientlByLinearAcceleration(
    const Vec3& linear_acceleration) {
  const auto q = orientation_;
  Mat3x4 jacobian{Mat3x4::Zero()};
  Vec3 residual{Vec3::Zero()};
  residual(0) = 2.0 * (q.x * q.z - q.w * q.y) - linear_acceleration.x() / 9.81;
  residual(1) = 2.0 * (q.w * q.x + q.y * q.z) - linear_acceleration.y() / 9.81;
  residual(2) =
      2.0 * (0.5 - q.x * q.x - q.y * q.y) - linear_acceleration.z() / 9.81;
  jacobian.row(0) << -2.0 * q.y, 2.0 * q.z, -2.0 * q.w, 2.0 * q.x;
  jacobian.row(1) << 2.0 * q.x, 2.0 * q.w, 2.0 * q.z, 2.0 * q.y;
  jacobian.row(2) << 0.0, -4.0 * q.x, -4.0 * q.y, 0.0;
  Vec4 dq_a_vec = jacobian.transpose() * residual;
  dq_a_vec.normalize();
  Quaternion dq_a(dq_a_vec(0), dq_a_vec(1), dq_a_vec(2), dq_a_vec(3));
  return dq_a;
}

Imu MadgwickFilter::ConvertFromBridge(const bridge::Imu& bridge_imu) {
  Imu imu;
  imu.time = bridge_imu.time;
  imu.linear_acceleration.x() = bridge_imu.linear_acceleration.x;
  imu.linear_acceleration.y() = bridge_imu.linear_acceleration.y;
  imu.linear_acceleration.z() = bridge_imu.linear_acceleration.z;
  imu.angular_velocity.x() = bridge_imu.angular_velocity.x;
  imu.angular_velocity.y() = bridge_imu.angular_velocity.y;
  imu.angular_velocity.z() = bridge_imu.angular_velocity.z;
  return imu;
}

}  // namespace madgwick_filter