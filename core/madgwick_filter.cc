#include "madgwick_filter.h"

#include <iostream>

namespace madgwick_filter {

MadgwickFilter::MadgwickFilter(const Parameters& parameters)
    : parameters_(parameters) {}

MadgwickFilter::~MadgwickFilter() {}

void MadgwickFilter::Update(const bridge::Imu& bridge_imu) {
  constexpr double kGravitationalAcceleration{9.81};
  const auto imu = ConvertFromBridge(bridge_imu);

  if (!condition_.is_bias_initialized) {
    constexpr int kMinNumImuData{50};
    imu_queue_.push_back(imu);
    if (imu_queue_.size() > kMinNumImuData)
      condition_.is_bias_initialized = InitializeBias();
    return;
  }

  const auto wm = imu.angular_velocity - gyro_bias_;
  Quaternion o_w;
  o_w.w = 0.0;
  o_w.x = wm.x();
  o_w.y = wm.y();
  o_w.z = wm.z();

  // Propagation via angular velocity
  if (prev_time_ == 0.0) {
    prev_time_ = imu.time;
    return;
  }

  const double dt = imu.time - prev_time_;
  Quaternion dq_w;
  dq_w = (orientation_ * o_w) * 0.5;

  // Compensate orientation using linear acceleration only when the norm
  // of acceleration is near the gravity.
  Eigen::Vector3d am = imu.linear_acceleration - acc_bias_;
  bool update_by_acceleration = false;
  if (std::abs(am.norm() - kGravitationalAcceleration) < 1e-2)
    update_by_acceleration = true;

  const auto q = orientation_;
  if (update_by_acceleration) {
    std::cerr << "Update By Acc.\n";

    Vec3 r_g;
    r_g(0) = 2.0 * (q.x * q.z - q.w * q.y) - am.x() / 9.81;
    r_g(1) = 2.0 * (q.w * q.x + q.y * q.z) - am.y() / 9.81;
    r_g(2) = 2.0 * (0.5 - q.x * q.x - q.y * q.y) - am.z() / 9.81;
    Eigen::Matrix<double, 3, 4> Jg;
    Jg(0, 0) = -2.0 * q.y;
    Jg(0, 1) = 2.0 * q.z;
    Jg(0, 2) = -2.0 * q.w;
    Jg(0, 3) = 2.0 * q.x;
    Jg(1, 0) = 2.0 * q.x;
    Jg(1, 1) = 2.0 * q.w;
    Jg(1, 2) = 2.0 * q.z;
    Jg(1, 3) = 2.0 * q.y;
    Jg(2, 0) = 0.0;
    Jg(2, 1) = -4.0 * q.x;
    Jg(2, 2) = -4.0 * q.y;
    Jg(2, 3) = 0.0;

    Eigen::Vector4d del_f = Jg.transpose() * r_g;
    del_f /= del_f.norm();

    const double kBeta = 0.2;
    Quaternion dq_a(-kBeta * del_f(0), -kBeta * del_f(1), -kBeta * del_f(2),
                    -kBeta * del_f(3));
    Quaternion dq_est = dq_w + dq_a;
    orientation_ = q + (dq_est * dt);
  } else {
    orientation_ = q + (dq_w * dt);
  }
  orientation_.normalize();

  prev_time_ = imu.time;
}

bridge::Orientation MadgwickFilter::GetOrientation() const {
  bridge::Orientation bridge_orientation;
  bridge_orientation.w = orientation_.w;
  bridge_orientation.x = orientation_.x;
  bridge_orientation.y = orientation_.y;
  bridge_orientation.z = orientation_.z;
  return bridge_orientation;
}

bool MadgwickFilter::InitializeBias() {
  // TODO(@): estimate acc bias
  Vec3 gyro_bias{Vec3::Zero()};
  for (const auto& imu : imu_queue_) {
    gyro_bias += imu.angular_velocity;
  }
  gyro_bias /= static_cast<double>(imu_queue_.size());
  gyro_bias_ = gyro_bias;
  return true;
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