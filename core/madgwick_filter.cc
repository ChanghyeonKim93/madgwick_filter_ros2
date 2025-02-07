#include "madgwick_filter.h"

#include <iostream>

namespace madgwick_filter {

MadgwickFilter::MadgwickFilter(const Parameters& parameters)
    : parameters_(parameters) {}

MadgwickFilter::~MadgwickFilter() {}

void MadgwickFilter::Update(const bridge::Imu& bridge_imu) {
  // TODO(@): make below as parameters
  const auto imu = ConvertFromBridge(bridge_imu);

  if (!condition_.is_bias_initialized) {
    constexpr int kMinNumImuData{50};
    imu_queue_.push_back(imu);
    if (imu_queue_.size() > kMinNumImuData)
      condition_.is_bias_initialized = InitializeGyroBias();
    return;
  }

  if (!condition_.is_initial_pose_estimated) {
    imu_queue_.push_back(imu);
  }

  if (previous_time_ == 0.0) {
    previous_time_ = imu.time;
    return;
  }

  const double grav_acc = parameters_.gravitational_acceleration;
  const double acc_convergence_rate =
      parameters_.convergence_rate.linear_acceleration;
  const double grav_acc_diff_threshold =
      parameters_.update_criteria.gravitation_diff_threshold;

  const double dt = imu.time - previous_time_;

  Quaternion dq;

  const Vec3 unbiased_gyro = imu.angular_velocity - gyro_bias_;
  const Quaternion dq_w = ComputeGradientByAngularVelocity(unbiased_gyro);
  dq = dq_w;

  Eigen::Vector3d unbiased_acc = imu.linear_acceleration - acc_bias_;
  const bool update_by_acc =
      std::abs(unbiased_acc.norm() - grav_acc) < grav_acc_diff_threshold;

  if (update_by_acc) {
    const Quaternion dq_a = ComputeGradientlByLinearAcceleration(unbiased_acc);
    dq = dq_w + dq_a * (-acc_convergence_rate);
  }
  orientation_ += dq * dt;
  orientation_.normalize();

  previous_time_ = imu.time;
}

void MadgwickFilter::SetImu(const bridge::Imu& bridge_imu) {
  Imu imu = ConvertFromBridge(bridge_imu);
  imu_queue_.push_back(imu);
}

void MadgwickFilter::SetMagneticField(
    const bridge::Vec3& bridge_magnetic_field) {
  Vec3 magnetic_field(bridge_magnetic_field.x, bridge_magnetic_field.y,
                      bridge_magnetic_field.z);
  magnetic_field_queue_.push_back(magnetic_field);
}

void MadgwickFilter::UpdateByMagnetometer(const bridge::Vec3& magnetic_field) {
  if (!parameters_.use_magnetometer) return;

  const double mag_convergence_rate = parameters_.convergence_rate.magnetometer;

  Vec3 m(magnetic_field.x, magnetic_field.y, magnetic_field.z);
  const Vec3 unbiased_magnetic_field = m - mag_bias_;

  // Rotate magnetic field
  const Quaternion dq_m =
      ComputeGradientlByMagnetometer(unbiased_magnetic_field);
  Quaternion dq = dq_m * (-mag_convergence_rate);

  orientation_ += dq;
  orientation_.normalize();
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
  const auto wm = angular_velocity;
  const Quaternion o_w(0.0, wm.x(), wm.y(), wm.z());
  const Quaternion dq_w = (orientation_ * o_w) * 0.5;
  return dq_w;
}

Quaternion MadgwickFilter::ComputeGradientlByLinearAcceleration(
    const Vec3& linear_acceleration) {
  auto a = linear_acceleration;
  a.normalize();
  const auto q = orientation_;
  Mat3x4 jacobian{Mat3x4::Zero()};
  Vec3 residual{Vec3::Zero()};
  residual(0) = 2.0 * (q.x * q.z - q.w * q.y) - a.x();
  residual(1) = 2.0 * (q.w * q.x + q.y * q.z) - a.y();
  residual(2) = 2.0 * (0.5 - q.x * q.x - q.y * q.y) - a.z();
  jacobian.row(0) << -2.0 * q.y, 2.0 * q.z, -2.0 * q.w, 2.0 * q.x;
  jacobian.row(1) << 2.0 * q.x, 2.0 * q.w, 2.0 * q.z, 2.0 * q.y;
  jacobian.row(2) << 0.0, -4.0 * q.x, -4.0 * q.y, 0.0;
  Vec4 dq_a_vec = jacobian.transpose() * residual;
  dq_a_vec.normalize();
  Quaternion dq_a(dq_a_vec(0), dq_a_vec(1), dq_a_vec(2), dq_a_vec(3));
  return dq_a;
}

Quaternion MadgwickFilter::ComputeGradientlByMagnetometer(
    const Vec3& magnetic_field) {
  // TODO(@): how to know the vector b?
  auto m = magnetic_field;

  Quaternion o_m(0.0, m.x(), m.y(), m.z());
  const auto h = orientation_ * o_m * orientation_.inverse();

  Vec3 b{Vec3::Zero()};  // Earth magnetic field at this latitude.
  b.x() = std::sqrt(h.x * h.x + h.y * h.y);
  b.z() = h.z;
  b.normalize();
  m.normalize();
  const auto q = orientation_;
  const auto bx = b.x(), bz = b.z();
  const auto qw = q.w, qx = q.x, qy = q.y, qz = q.z;
  Mat3x4 jacobian{Mat3x4::Zero()};
  Vec3 residual{Vec3::Zero()};
  residual(0) =
      2 * bx * (0.5 - qy * qy - qz * qz) + 2 * bz * (qx * qz - qw * qy) - m.x();
  residual(1) =
      2 * bx * (qx * qy - qw * qz) + 2 * bz * (qw * qx + qy * qz) - m.y();
  residual(2) =
      2 * bx * (qw * qy + qx * qz) + 2 * bz * (0.5 - qx * qx - qy * qy) - m.z();

  jacobian(0, 0) = -2 * bz * qy;
  jacobian(0, 1) = 2 * bz * qz;
  jacobian(0, 2) = -4 * bx * qy - 2 * bz * qw;
  jacobian(0, 3) = -4 * bx * qz + 2 * bz * qx;

  jacobian(1, 0) = -2 * bx * qz + 2 * bz * qx;
  jacobian(1, 1) = 2 * bx * qy + 2 * bz * qw;
  jacobian(1, 2) = 2 * bx * qx + 2 * bz * qz;
  jacobian(1, 3) = -2 * bx * qw + 2 * bz * qy;

  jacobian(2, 0) = 2 * bx * qy;
  jacobian(2, 1) = 2 * bx * qz - 4 * bz * qx;
  jacobian(2, 2) = 2 * bx * qw - 4 * bz * qy;
  jacobian(2, 3) = 2 * bx * qx;
  Vec4 dq_m_vec = jacobian.transpose() * residual;
  dq_m_vec.normalize();
  Quaternion dq_m(dq_m_vec(0), dq_m_vec(1), dq_m_vec(2), dq_m_vec(3));
  return dq_m;
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